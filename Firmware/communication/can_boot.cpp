#include "can_boot.hpp"

#include "communication/bootloader.hpp"
#include "MotorControl/odrive_main.h"

#include <stddef.h>
#include <cstring>

#include <stm32f4xx_hal.h>

namespace {

enum class TransferState : uint8_t {
    idle,
    awaiting_size,
    stopping,
    receiving
};

uint8_t packet_buffer[OTA_CHUNK_SIZE];
TransferState transfer_state = TransferState::idle;
uint8_t image_type = 0;
uint32_t firmware_size = 0;
uint32_t firmware_crc32 = 0;
uint32_t firmware_chunk_count = 0;
uint32_t next_sequence = 0;
uint32_t packet_sequence = 0;
uint32_t last_packet_sequence = 0;
uint32_t last_packet_crc = 0;
uint16_t next_frame_index = 0;
bool packet_open = false;
bool last_packet_valid = false;
bool duplicate_packet = false;
bool reset_pending = false;

uint16_t read_u16(const uint8_t* data) {
    return static_cast<uint16_t>(data[0]) |
           (static_cast<uint16_t>(data[1]) << 8);
}

uint32_t read_u32(const uint8_t* data) {
    return static_cast<uint32_t>(data[0]) |
           (static_cast<uint32_t>(data[1]) << 8) |
           (static_cast<uint32_t>(data[2]) << 16) |
           (static_cast<uint32_t>(data[3]) << 24);
}

uint32_t crc32(const uint8_t* data, size_t length) {
    uint32_t crc = 0xFFFFFFFF;
    for (size_t i = 0; i < length; ++i) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; ++bit)
            crc = (crc >> 1) ^ ((crc & 1) ? 0xEDB88320 : 0);
    }
    return ~crc;
}

bool matches_installed_image() {
    // NVM can change outside OTA. Skip only when the complete destination state
    // already matches: A contains the image and B is erased.
    const uint32_t address = image_type == OtaCanProtocol::APPLICATION ?
        APPLICATION_START : APPLICATION_CONFIG_A_START;
    const uint8_t* data = reinterpret_cast<const uint8_t*>(address);
    const uint32_t capacity = image_type == OtaCanProtocol::APPLICATION ?
        APPLICATION_SIZE : APPLICATION_CONFIG_SIZE;
    if (crc32(data, firmware_size) != firmware_crc32)
        return false;
    for (uint32_t i = firmware_size; i < capacity; ++i)
        if (data[i] != 0xFF)
            return false;
    if (image_type == OtaCanProtocol::PARAMETERS) {
        const uint8_t* sector_b =
            reinterpret_cast<const uint8_t*>(APPLICATION_CONFIG_B_START);
        for (uint32_t i = 0; i < APPLICATION_CONFIG_SIZE; ++i)
            if (sector_b[i] != 0xFF)
                return false;
    }
    return true;
}

void make_response(can_Message_t& response, OtaCanProtocol::Status status,
                   uint16_t sequence = 0, uint16_t expected_sequence = 0) {
    response = can_Message_t();
    response.id = OtaCanProtocol::CAN_ID;
    response.isExt = true;
    response.len = 8;
    response.buf[0] = OtaCanProtocol::FRAME_ACK;
    response.buf[1] = static_cast<uint8_t>(status);
    response.buf[2] = static_cast<uint8_t>(sequence);
    response.buf[3] = static_cast<uint8_t>(sequence >> 8);
    response.buf[4] = static_cast<uint8_t>(expected_sequence);
    response.buf[5] = static_cast<uint8_t>(expected_sequence >> 8);
}

void request_axes_idle() {
    for (Axis* axis : axes) {
        if (axis)
            axis->requested_state_ = Axis::AXIS_STATE_IDLE;
    }
}

bool axes_stopped() {
    for (Axis* axis : axes) {
        if (!axis || axis->current_state_ != Axis::AXIS_STATE_IDLE ||
            axis->motor_.armed_state_ != Motor::ARMED_STATE_DISARMED) {
            return false;
        }
    }
    return true;
}

bool erase_flash_sector(uint32_t sector) {
    FLASH_EraseInitTypeDef erase_info = {};
    erase_info.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_info.Banks = 0;
    erase_info.Sector = sector;
    erase_info.NbSectors = 1;
    erase_info.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
    uint32_t sector_error = 0;
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_info, &sector_error);
    HAL_FLASH_Lock();
    return status == HAL_OK && sector_error == 0xFFFFFFFF;
}

bool erase_application_buffer() {
    FLASH_EraseInitTypeDef erase_info = {};
    erase_info.TypeErase = FLASH_TYPEERASE_SECTORS;
    erase_info.Banks = 0;
    erase_info.Sector = FLASH_SECTOR_7;
    erase_info.NbSectors = 3;
    erase_info.VoltageRange = FLASH_VOLTAGE_RANGE_3;

    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
    uint32_t sector_error = 0;
    const HAL_StatusTypeDef status = HAL_FLASHEx_Erase(&erase_info, &sector_error);
    HAL_FLASH_Lock();
    return status == HAL_OK && sector_error == 0xFFFFFFFF;
}

bool write_packet(uint32_t sequence) {
    const uintptr_t address = APPLICATION_BUFFER_START + sequence * OTA_CHUNK_SIZE;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

    for (uint32_t offset = 0; offset < OTA_CHUNK_SIZE; offset += sizeof(uint32_t)) {
        uint32_t word;
        std::memcpy(&word, &packet_buffer[offset], sizeof(word));
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD, address + offset, word) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }

    HAL_FLASH_Lock();
    return true;
}

bool save_pending_metadata() {
    OtaMetadata metadata = {
        image_type == OtaCanProtocol::APPLICATION ? OTA_METADATA_MAGIC : OTA_CONFIG_MAGIC,
        OTA_METADATA_FORMAT,
        image_type,
        firmware_size,
        firmware_crc32,
        firmware_chunk_count,
        0,
        OTA_STATE_EMPTY
    };
    metadata.metadata_crc32 =
        crc32(reinterpret_cast<const uint8_t*>(&metadata),
              offsetof(OtaMetadata, metadata_crc32));

    if (!erase_flash_sector(FLASH_SECTOR_3))
        return false;

    const uintptr_t address = BOOTLOADER_DATA_START;
    const uint32_t* words = reinterpret_cast<const uint32_t*>(&metadata);
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
    for (uint32_t i = 0; i < 7; ++i) {
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              address + i * sizeof(uint32_t), words[i]) != HAL_OK) {
            HAL_FLASH_Lock();
            return false;
        }
    }
    if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                          address + offsetof(OtaMetadata, state),
                          OTA_STATE_PENDING) != HAL_OK) {
        HAL_FLASH_Lock();
        return false;
    }
    HAL_FLASH_Lock();
    return true;
}

void clear_packet() {
    std::memset(packet_buffer, 0xFF, sizeof(packet_buffer));
    next_frame_index = 0;
}

void abort_packet(can_Message_t& response, OtaCanProtocol::Status status,
                  uint16_t sequence) {
    packet_open = false;
    make_response(response, status, sequence, static_cast<uint16_t>(next_sequence));
}

void begin_stop(can_Message_t& response) {
    transfer_state = TransferState::stopping;
    request_axes_idle();
    make_response(response, OtaCanProtocol::STATUS_STOPPING);
}

bool handle_upgrade_request(const can_Message_t& message, can_Message_t& response) {
    const uint8_t type = message.buf[1];
    const uint32_t count = read_u16(&message.buf[2]);
    if (message.len != 8 || !valid_image_size(type, count * OTA_CHUNK_SIZE)) {
        make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR);
        return true;
    }

    if (transfer_state == TransferState::receiving) {
        make_response(response, OtaCanProtocol::STATUS_BUSY);
        return true;
    }

    image_type = type;
    firmware_chunk_count = count;
    firmware_crc32 = read_u32(&message.buf[4]);
    transfer_state = TransferState::awaiting_size;
    make_response(response, OtaCanProtocol::STATUS_NEED_SIZE);
    return true;
}

bool handle_metadata(const can_Message_t& message, can_Message_t& response) {
    const uint32_t size = read_u32(&message.buf[4]);
    if (message.len != 8 || message.buf[1] != image_type ||
        read_u16(&message.buf[2]) != firmware_chunk_count ||
        !valid_image_size(image_type, size) ||
        (size + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE != firmware_chunk_count) {
        make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR);
    } else if (transfer_state == TransferState::stopping) {
        make_response(response, size == firmware_size ? OtaCanProtocol::STATUS_STOPPING :
                      OtaCanProtocol::STATUS_SIZE_ERROR);
    } else if (transfer_state != TransferState::awaiting_size) {
        make_response(response, OtaCanProtocol::STATUS_BUSY);
    } else {
        firmware_size = size;
        if (matches_installed_image())
            make_response(response, OtaCanProtocol::STATUS_ALREADY_CURRENT);
        else
            begin_stop(response);
    }
    return true;
}

bool handle_status_request(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 1) {
        make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR);
        return true;
    }
    if (transfer_state == TransferState::receiving && next_sequence == 0 && !packet_open) {
        make_response(response, OtaCanProtocol::STATUS_STOPPED);
        return true;
    }
    if (transfer_state != TransferState::stopping) {
        make_response(response, OtaCanProtocol::STATUS_BUSY);
        return true;
    }

    request_axes_idle();
    if (axes_stopped()) {
        if (!erase_application_buffer()) {
            transfer_state = TransferState::idle;
            make_response(response, OtaCanProtocol::STATUS_FLASH_ERROR);
            return true;
        }
        transfer_state = TransferState::receiving;
        next_sequence = 0;
        packet_open = false;
        last_packet_valid = false;
        make_response(response, OtaCanProtocol::STATUS_STOPPED);
    } else {
        make_response(response, OtaCanProtocol::STATUS_STOPPING);
    }
    return true;
}

bool handle_start(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 8 || message.buf[1] != 0 ||
        transfer_state != TransferState::receiving) {
        make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR);
        return true;
    }

    const uint16_t sequence = read_u16(&message.buf[2]);
    if (read_u32(&message.buf[4]) != firmware_size) {
        make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR, sequence,
                      static_cast<uint16_t>(next_sequence));
        return true;
    }
    if (sequence == next_sequence) {
        duplicate_packet = false;
    } else if (last_packet_valid && sequence == last_packet_sequence) {
        duplicate_packet = true;
    } else {
        make_response(response, OtaCanProtocol::STATUS_SEQUENCE_ERROR, sequence,
                      static_cast<uint16_t>(next_sequence));
        return true;
    }
    if (sequence >= firmware_chunk_count) {
        make_response(response, OtaCanProtocol::STATUS_SEQUENCE_ERROR, sequence,
                      static_cast<uint16_t>(next_sequence));
        return true;
    }

    packet_sequence = sequence;
    packet_open = true;
    clear_packet();
    make_response(response, OtaCanProtocol::STATUS_READY, sequence,
                  static_cast<uint16_t>(next_sequence));
    return true;
}

bool handle_data(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 8 || message.buf[0] != OtaCanProtocol::FRAME_DATA ||
        message.buf[1] != next_frame_index ||
        next_frame_index >= (OTA_CHUNK_SIZE + 5) / 6) {
        abort_packet(response, OtaCanProtocol::STATUS_RETRY_DATA,
                     static_cast<uint16_t>(packet_sequence));
        return true;
    }

    const uint32_t offset = static_cast<uint32_t>(next_frame_index) * 6;
    const uint32_t bytes_to_copy =
        OTA_CHUNK_SIZE - offset < 6 ? OTA_CHUNK_SIZE - offset : 6;
    std::memcpy(&packet_buffer[offset], &message.buf[2], bytes_to_copy);
    if (bytes_to_copy < 6) {
        for (uint32_t i = bytes_to_copy; i < 6; ++i) {
            if (message.buf[2 + i] != 0xFF) {
                abort_packet(response, OtaCanProtocol::STATUS_RETRY_DATA,
                             static_cast<uint16_t>(packet_sequence));
                return true;
            }
        }
    }
    ++next_frame_index;
    return true;
}

bool handle_end(const can_Message_t& message, can_Message_t& response) {
    const uint16_t sequence = message.len >= 3 ? read_u16(&message.buf[1]) : 0;
    if (message.len != 7 || message.buf[0] != OtaCanProtocol::FRAME_END ||
        sequence != packet_sequence ||
        next_frame_index != (OTA_CHUNK_SIZE + 5) / 6) {
        abort_packet(response, OtaCanProtocol::STATUS_RETRY_DATA, sequence);
        return true;
    }

    const uint32_t received_crc = read_u32(&message.buf[3]);
    packet_open = false;
    if (received_crc != crc32(packet_buffer, sizeof(packet_buffer))) {
        abort_packet(response, OtaCanProtocol::STATUS_RETRY_CHECKSUM, sequence);
        return true;
    }
    const uint32_t remaining = firmware_size - sequence * OTA_CHUNK_SIZE;
    for (uint32_t i = remaining; i < OTA_CHUNK_SIZE; ++i) {
        if (packet_buffer[i] != 0xFF) {
            abort_packet(response, OtaCanProtocol::STATUS_RETRY_DATA, sequence);
            return true;
        }
    }

    if (duplicate_packet) {
        if (sequence != last_packet_sequence || received_crc != last_packet_crc) {
            abort_packet(response, OtaCanProtocol::STATUS_RETRY_CHECKSUM, sequence);
            return true;
        }
    } else {
        if (!write_packet(sequence)) {
            transfer_state = TransferState::idle;
            abort_packet(response, OtaCanProtocol::STATUS_FLASH_ERROR, sequence);
            return true;
        }
        last_packet_sequence = sequence;
        last_packet_crc = received_crc;
        last_packet_valid = true;
        ++next_sequence;
    }

    const bool final_packet = next_sequence == firmware_chunk_count;
    if (final_packet &&
        crc32(reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START),
              firmware_size) != firmware_crc32) {
        transfer_state = TransferState::idle;
        make_response(response, OtaCanProtocol::STATUS_IMAGE_CHECKSUM, sequence,
                      static_cast<uint16_t>(next_sequence));
        return true;
    }

    if (final_packet) {
        const uint8_t* staged = reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START);
        if (image_type == OtaCanProtocol::APPLICATION &&
            !valid_app_vectors(read_u32(staged), read_u32(staged + 4), firmware_size)) {
            transfer_state = TransferState::idle;
            make_response(response, OtaCanProtocol::STATUS_SIZE_ERROR, sequence);
            return true;
        }
        if (!save_pending_metadata()) {
            transfer_state = TransferState::idle;
            make_response(response, OtaCanProtocol::STATUS_FLASH_ERROR, sequence,
                          static_cast<uint16_t>(next_sequence));
            return true;
        }
        reset_pending = true;
        make_response(response, OtaCanProtocol::STATUS_COMPLETE, sequence,
                      static_cast<uint16_t>(next_sequence));
    } else {
        make_response(response, OtaCanProtocol::STATUS_ACCEPTED, sequence,
                      static_cast<uint16_t>(next_sequence));
    }
    return true;
}

}

bool FirmwareOTA::handle(const can_Message_t& message, can_Message_t& response) {
    response.len = 0;
    if (!message.isExt || message.id != OtaCanProtocol::CAN_ID)
        return false;
    if (message.len == 0 || message.rtr)
        return true;

    switch (message.buf[0]) {
        case OtaCanProtocol::FRAME_UPGRADE_REQUEST:
            return handle_upgrade_request(message, response);
        case OtaCanProtocol::FRAME_STATUS_REQUEST:
            return handle_status_request(message, response);
        case OtaCanProtocol::FRAME_METADATA:
            return handle_metadata(message, response);
        case OtaCanProtocol::FRAME_START:
            return handle_start(message, response);
        case OtaCanProtocol::FRAME_END:
            if (transfer_state == TransferState::receiving && packet_open)
                return handle_end(message, response);
            return true;
        case OtaCanProtocol::FRAME_DATA:
            if (transfer_state == TransferState::receiving && packet_open)
                return handle_data(message, response);
            return true;
        default:
            return true;
    }
}

bool FirmwareOTA::reset_after_response() {
    if (!reset_pending)
        return false;

    reset_pending = false;
    transfer_state = TransferState::idle;
    return true;
}
