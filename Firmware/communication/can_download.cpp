#include "can_download.hpp"

#include "communication/bootloader.hpp"
#include "MotorControl/odrive_main.h"
#include "MotorControl/nvm.h"

#include <stddef.h>
#include <cstring>

#include <stm32f4xx_hal.h>

namespace {

enum class TransferState : uint8_t {
    idle,
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
uint8_t last_frame_data[6];
bool last_frame_valid = false;
bool last_chunk_crc_failure_valid = false;
uint32_t last_chunk_crc_failure_sequence = 0;
uint32_t last_chunk_crc_failure_expected = 0;
uint32_t last_chunk_crc_failure_actual = 0;
bool last_image_crc_response_valid = false;
uint32_t last_image_crc_request = 0;
uint32_t last_image_crc_response = 0;
uint8_t last_image_crc_response_length = 0;
bool reset_pending = false;

constexpr uint32_t FRAMES_PER_PACKET = (OTA_CHUNK_SIZE + 5) / 6;

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

void make_ready_response(can_Message_t& response, OtaCanProtocol::ReadyStatus status,
                         uint32_t chunk_count) {
    response = can_Message_t();
    response.id = OtaCanProtocol::RESPONSE_CAN_ID;
    response.isExt = true;
    response.len = 8;
    response.buf[0] = OtaCanProtocol::FRAME_READY;
    response.buf[1] = static_cast<uint8_t>(status);
    response.buf[2] = static_cast<uint8_t>(chunk_count);
    response.buf[3] = static_cast<uint8_t>(chunk_count >> 8);
    response.buf[4] = 0;
    response.buf[5] = 0;
    response.buf[6] = 0;
    response.buf[7] = 0;
}

void make_frame_ack(can_Message_t& response, uint8_t frame_index) {
    response = can_Message_t();
    response.id = OtaCanProtocol::RESPONSE_CAN_ID;
    response.isExt = true;
    response.len = 7;
    response.buf[0] = OtaCanProtocol::FRAME_DATA_ACK;
    response.buf[1] = frame_index;
    for (uint8_t i = 2; i < response.len; ++i)
        response.buf[i] = 0;
}

void make_crc_response(can_Message_t& response, uint8_t opcode, uint32_t crc) {
    response = can_Message_t();
    response.id = OtaCanProtocol::RESPONSE_CAN_ID;
    response.isExt = true;
    response.len = 5;
    response.buf[0] = opcode;
    response.buf[1] = static_cast<uint8_t>(crc);
    response.buf[2] = static_cast<uint8_t>(crc >> 8);
    response.buf[3] = static_cast<uint8_t>(crc >> 16);
    response.buf[4] = static_cast<uint8_t>(crc >> 24);
}

void make_crc_error_response(can_Message_t& response, uint8_t opcode) {
    response = can_Message_t();
    response.id = OtaCanProtocol::RESPONSE_CAN_ID;
    response.isExt = true;
    response.len = 1;
    response.buf[0] = opcode;
}

void make_version_response(can_Message_t& response) {
    response = can_Message_t();
    response.id = OtaCanProtocol::RESPONSE_CAN_ID;
    response.isExt = true;
    response.len = 5;
    response.buf[0] = OtaCanProtocol::FRAME_VERSION_RESPONSE;
    response.buf[1] = fw_version_major_;
    response.buf[2] = fw_version_minor_;
    response.buf[3] = fw_version_revision_;
    response.buf[4] = fw_version_unreleased_;
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

bool install_parameter_image(uint32_t& installed_crc) {
    if (!erase_flash_sector(FLASH_SECTOR_10) ||
        !erase_flash_sector(FLASH_SECTOR_11)) {
        NVM_init();
        return false;
    }

    const uint8_t* source =
        reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START);
    const uint32_t word_count = (firmware_size + 3) / 4;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);
    for (uint32_t i = 0; i < word_count; ++i) {
        uint32_t word = 0xFFFFFFFF;
        const uint32_t offset = i * 4;
        const uint32_t bytes = firmware_size - offset < 4 ?
            firmware_size - offset : 4;
        std::memcpy(&word, source + offset, bytes);
        if (HAL_FLASH_Program(FLASH_TYPEPROGRAM_WORD,
                              APPLICATION_CONFIG_A_START + offset, word) != HAL_OK) {
            HAL_FLASH_Lock();
            NVM_init();
            return false;
        }
    }
    HAL_FLASH_Lock();

    installed_crc = crc32(
        reinterpret_cast<const uint8_t*>(APPLICATION_CONFIG_A_START), firmware_size);
    const bool nvm_ready = NVM_init() == 0;
    return nvm_ready && installed_crc == firmware_crc32;
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
    last_frame_valid = false;
}

void begin_packet() {
    packet_sequence = next_sequence;
    packet_open = true;
    last_packet_valid = false;
    last_chunk_crc_failure_valid = false;
    clear_packet();
}

void restart_packet() {
    packet_open = false;
    next_frame_index = 0;
    last_frame_valid = false;
}

bool handle_upgrade_request(const can_Message_t& message, can_Message_t& response,
                            uint8_t type) {
    if (message.len != 4) {
        make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        return true;
    }
    const uint32_t size = static_cast<uint32_t>(message.buf[1]) |
                          (static_cast<uint32_t>(message.buf[2]) << 8) |
                          (static_cast<uint32_t>(message.buf[3]) << 16);
    if (!image_fits_flash(type, size)) {
        make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        return true;
    }

    if (transfer_state == TransferState::stopping) {
        if (image_type == type && firmware_size == size) {
            make_ready_response(response, OtaCanProtocol::READY_PENDING,
                                firmware_chunk_count);
        } else {
            make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        }
        return true;
    }
    if (transfer_state == TransferState::receiving) {
        if (image_type == type && firmware_size == size &&
            next_sequence == 0 && !packet_open) {
            make_ready_response(response, OtaCanProtocol::READY_SUCCESS,
                                firmware_chunk_count);
            return true;
        }
        make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        return true;
    }

    image_type = type;
    firmware_size = size;
    firmware_crc32 = 0;
    firmware_chunk_count = (size + OTA_CHUNK_SIZE - 1) / OTA_CHUNK_SIZE;
    last_packet_valid = false;
    last_chunk_crc_failure_valid = false;
    last_image_crc_response_valid = false;
    reset_pending = false;
    transfer_state = TransferState::stopping;
    request_axes_idle();
    response.len = 0;
    return true;
}

bool prepare_transfer(can_Message_t& response) {
    request_axes_idle();
    if (!axes_stopped())
        return false;

    if (!erase_application_buffer()) {
        transfer_state = TransferState::idle;
        make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        return true;
    }

    transfer_state = TransferState::receiving;
    next_sequence = 0;
    packet_sequence = 0;
    packet_open = false;
    last_packet_valid = false;
    clear_packet();
    make_ready_response(response, OtaCanProtocol::READY_SUCCESS,
                        firmware_chunk_count);
    return true;
}

bool handle_data(const can_Message_t& message, can_Message_t& response) {
    const uint8_t frame_index = message.len >= 2 ? message.buf[1] : 0xFF;
    if (transfer_state != TransferState::receiving ||
        next_sequence >= firmware_chunk_count || message.len != 8) {
        make_frame_ack(response, frame_index);
        return true;
    }

    if (!packet_open)
        begin_packet();

    if (last_frame_valid &&
        static_cast<uint16_t>(frame_index) + 1 == next_frame_index) {
        make_frame_ack(response, frame_index);
        return true;
    }
    if (frame_index != next_frame_index || next_frame_index >= FRAMES_PER_PACKET) {
        make_frame_ack(response, frame_index);
        return true;
    }

    const uint32_t offset = static_cast<uint32_t>(next_frame_index) * 6;
    const uint32_t bytes_to_copy =
        OTA_CHUNK_SIZE - offset < 6 ? OTA_CHUNK_SIZE - offset : 6;
    if (bytes_to_copy < 6) {
        for (uint32_t i = bytes_to_copy; i < 6; ++i) {
            if (message.buf[2 + i] != 0xFF) {
                make_frame_ack(response, frame_index);
                return true;
            }
        }
    }
    std::memcpy(&packet_buffer[offset], &message.buf[2], bytes_to_copy);
    std::memcpy(last_frame_data, &message.buf[2], sizeof(last_frame_data));
    last_frame_valid = true;
    ++next_frame_index;
    make_frame_ack(response, frame_index);
    return true;
}

bool handle_chunk_crc_request(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 5) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        return true;
    }
    const uint32_t expected_crc = read_u32(&message.buf[1]);
    if (!packet_open) {
        if (last_chunk_crc_failure_valid &&
            last_chunk_crc_failure_sequence == next_sequence &&
            last_chunk_crc_failure_expected == expected_crc) {
            make_crc_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE,
                              last_chunk_crc_failure_actual);
        } else if (last_packet_valid &&
                   next_sequence == last_packet_sequence + 1 &&
                   expected_crc == last_packet_crc) {
            make_crc_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE,
                              last_packet_crc);
        } else {
            make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        }
        return true;
    }
    if (transfer_state != TransferState::receiving) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        return true;
    }
    if (packet_sequence != next_sequence) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        return true;
    }

    const uint32_t computed_packet_crc = crc32(packet_buffer, sizeof(packet_buffer));
    if (next_frame_index != FRAMES_PER_PACKET) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        restart_packet();
        return true;
    }
    if (expected_crc != computed_packet_crc) {
        last_chunk_crc_failure_valid = true;
        last_chunk_crc_failure_sequence = packet_sequence;
        last_chunk_crc_failure_expected = expected_crc;
        last_chunk_crc_failure_actual = computed_packet_crc;
        restart_packet();
        make_crc_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE,
                          computed_packet_crc);
        return true;
    }

    const uint32_t remaining = firmware_size - packet_sequence * OTA_CHUNK_SIZE;
    for (uint32_t i = remaining; i < OTA_CHUNK_SIZE; ++i) {
        if (packet_buffer[i] != 0xFF) {
            make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
            restart_packet();
            return true;
        }
    }

    if (!write_packet(packet_sequence)) {
        transfer_state = TransferState::idle;
        packet_open = false;
        make_crc_error_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE);
        return true;
    }
    last_packet_sequence = packet_sequence;
    last_packet_crc = computed_packet_crc;
    last_packet_valid = true;
    ++next_sequence;
    packet_open = false;
    next_frame_index = 0;
    last_frame_valid = false;
    last_chunk_crc_failure_valid = false;
    make_crc_response(response, OtaCanProtocol::FRAME_CHUNK_CRC_RESPONSE,
                      computed_packet_crc);
    return true;
}

bool handle_image_crc_request(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 5) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE);
        return true;
    }
    const uint32_t expected_crc = read_u32(&message.buf[1]);
    if (last_image_crc_response_valid &&
        expected_crc == last_image_crc_request &&
        transfer_state != TransferState::receiving) {
        if (last_image_crc_response_length == 5)
            make_crc_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE,
                              last_image_crc_response);
        else
            make_crc_error_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE);
        return true;
    }
    if (transfer_state != TransferState::receiving ||
        packet_open || next_sequence != firmware_chunk_count) {
        make_crc_error_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE);
        return true;
    }

    uint32_t actual_crc =
        crc32(reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START),
              firmware_size);
    last_image_crc_response_valid = true;
    last_image_crc_request = expected_crc;
    last_image_crc_response = actual_crc;
    if (actual_crc != expected_crc) {
        transfer_state = TransferState::idle;
        last_image_crc_response_length = 5;
        make_crc_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE,
                          actual_crc);
        return true;
    }

    firmware_crc32 = expected_crc;
    if (image_type == OtaCanProtocol::APPLICATION) {
        const uint8_t* staged =
            reinterpret_cast<const uint8_t*>(APPLICATION_BUFFER_START);
        if (!valid_app_reset_entry(read_u32(staged + 4)) ||
            !save_pending_metadata()) {
            transfer_state = TransferState::idle;
            last_image_crc_response_length = 1;
            make_crc_error_response(response,
                                    OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE);
            return true;
        }
        reset_pending = true;
    } else {
        uint32_t installed_crc = actual_crc;
        if (!install_parameter_image(installed_crc) ||
            installed_crc != expected_crc) {
            transfer_state = TransferState::idle;
            last_image_crc_response_length = 1;
            make_crc_error_response(response,
                                    OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE);
            return true;
        }
        actual_crc = installed_crc;
        transfer_state = TransferState::idle;
        last_image_crc_response = actual_crc;
    }
    last_image_crc_response_length = 5;
    make_crc_response(response, OtaCanProtocol::FRAME_IMAGE_CRC_RESPONSE,
                      actual_crc);
    return true;
}

}

bool FirmwareOTA::handle(const can_Message_t& message, can_Message_t& response) {
    response.len = 0;
    if (!message.isExt || message.id != OtaCanProtocol::CAN_ID)
        return false;
    if (message.len == 0 || message.rtr) {
        make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
        return true;
    }

    switch (message.buf[0]) {
        case OtaCanProtocol::FRAME_APPLICATION_REQUEST:
            return handle_upgrade_request(message, response,
                                          OtaCanProtocol::APPLICATION);
        case OtaCanProtocol::FRAME_PARAMETERS_REQUEST:
            return handle_upgrade_request(message, response,
                                          OtaCanProtocol::PARAMETERS);
        case OtaCanProtocol::FRAME_CHUNK_CRC_REQUEST:
            return handle_chunk_crc_request(message, response);
        case OtaCanProtocol::FRAME_IMAGE_CRC_REQUEST:
            return handle_image_crc_request(message, response);
        case OtaCanProtocol::FRAME_VERSION_REQUEST:
            make_version_response(response);
            return true;
        case OtaCanProtocol::FRAME_DATA:
            return handle_data(message, response);
        case OtaCanProtocol::FRAME_READY:
            make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
            return true;
        default:
            make_ready_response(response, OtaCanProtocol::READY_REJECTED, 0);
            return true;
    }
}

bool FirmwareOTA::process(can_Message_t& response) {
    response.len = 0;
    if (transfer_state != TransferState::stopping)
        return false;
    return prepare_transfer(response);
}

bool FirmwareOTA::reset_after_response() {
    if (!reset_pending)
        return false;

    reset_pending = false;
    transfer_state = TransferState::idle;
    return true;
}
