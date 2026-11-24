#include "bootloader.hpp"

#include <cstring>

#include <stm32f4xx_hal.h>

namespace {

enum class ReceiveState : uint8_t {
    waiting_for_start,
    receiving_data
};

uint8_t packet_buffer[FirmwareDownload::CHUNK_SIZE];
ReceiveState receive_state = ReceiveState::waiting_for_start;
bool transfer_active = false;
bool duplicate_packet = false;
bool last_packet_valid = false;
uint32_t image_size = 0;
uint32_t chunk_count = 0;
uint32_t next_sequence = 0;
uint32_t packet_sequence = 0;
uint32_t last_packet_sequence = 0;
uint32_t last_packet_crc = 0;
uint16_t next_frame_index = 0;

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

void make_response(can_Message_t& response, FirmwareDownload::Status status,
                   uint16_t sequence, uint16_t expected_sequence) {
    response = can_Message_t();
    response.id = FirmwareDownload::CAN_ID;
    response.isExt = true;
    response.len = 8;
    response.buf[0] = FirmwareDownload::FRAME_ACK;
    response.buf[1] = static_cast<uint8_t>(status);
    response.buf[2] = static_cast<uint8_t>(sequence);
    response.buf[3] = static_cast<uint8_t>(sequence >> 8);
    response.buf[4] = static_cast<uint8_t>(expected_sequence);
    response.buf[5] = static_cast<uint8_t>(expected_sequence >> 8);
}

uint16_t expected_sequence_for_response() {
    return static_cast<uint16_t>(next_sequence);
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
    return status == HAL_OK;
}

bool write_packet(uint32_t sequence) {
    const uintptr_t address = APPLICATION_BUFFER_START + sequence * FirmwareDownload::CHUNK_SIZE;
    HAL_FLASH_Unlock();
    __HAL_FLASH_CLEAR_FLAG(FLASH_FLAG_EOP | FLASH_FLAG_OPERR | FLASH_FLAG_WRPERR |
                           FLASH_FLAG_PGAERR | FLASH_FLAG_PGSERR | FLASH_FLAG_PGPERR);

    for (uint32_t offset = 0; offset < FirmwareDownload::CHUNK_SIZE; offset += sizeof(uint32_t)) {
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

void clear_packet() {
    std::memset(packet_buffer, 0xFF, sizeof(packet_buffer));
    next_frame_index = 0;
    receive_state = ReceiveState::receiving_data;
}

void abort_packet(can_Message_t& response, FirmwareDownload::Status status,
                  uint16_t sequence) {
    receive_state = ReceiveState::waiting_for_start;
    make_response(response, status, sequence, expected_sequence_for_response());
}

bool handle_start(const can_Message_t& message, can_Message_t& response) {
    if (message.len != 8 || (message.buf[1] & ~0x01) != 0) {
        make_response(response, FirmwareDownload::STATUS_SIZE_ERROR,
                      message.len >= 4 ? read_u16(&message.buf[2]) : 0,
                      expected_sequence_for_response());
        return true;
    }

    const bool reset = (message.buf[1] & 0x01) != 0;
    const uint16_t sequence = read_u16(&message.buf[2]);
    const uint32_t requested_image_size = read_u32(&message.buf[4]);
    const uint32_t requested_chunk_count =
        (requested_image_size + FirmwareDownload::CHUNK_SIZE - 1) /
        FirmwareDownload::CHUNK_SIZE;

    if (requested_image_size == 0 ||
        requested_image_size > APPLICATION_BUFFER_SIZE ||
        requested_chunk_count > FirmwareDownload::MAX_CHUNKS) {
        make_response(response, FirmwareDownload::STATUS_SIZE_ERROR, sequence,
                      expected_sequence_for_response());
        return true;
    }

    if (reset) {
        if (sequence != 0) {
            make_response(response, FirmwareDownload::STATUS_SEQUENCE_ERROR, sequence,
                          expected_sequence_for_response());
            return true;
        }
        transfer_active = false;
        last_packet_valid = false;
        next_sequence = 0;
        receive_state = ReceiveState::waiting_for_start;
        if (!erase_application_buffer()) {
            make_response(response, FirmwareDownload::STATUS_FLASH_ERROR, sequence,
                          expected_sequence_for_response());
            return true;
        }

        transfer_active = true;
        last_packet_valid = false;
        next_sequence = 0;
        image_size = requested_image_size;
        chunk_count = requested_chunk_count;
    } else if (!transfer_active) {
        if (sequence != 0) {
            make_response(response, FirmwareDownload::STATUS_SEQUENCE_ERROR, sequence,
                          expected_sequence_for_response());
            return true;
        }
        if (!erase_application_buffer()) {
            make_response(response, FirmwareDownload::STATUS_FLASH_ERROR, sequence,
                          expected_sequence_for_response());
            return true;
        }
        transfer_active = true;
        last_packet_valid = false;
        next_sequence = 0;
        image_size = requested_image_size;
        chunk_count = requested_chunk_count;
    }

    if (requested_image_size != image_size) {
        make_response(response, FirmwareDownload::STATUS_SIZE_ERROR, sequence,
                      expected_sequence_for_response());
        return true;
    }

    if (sequence == next_sequence) {
        duplicate_packet = false;
    } else if (last_packet_valid && sequence == last_packet_sequence) {
        duplicate_packet = true;
    } else {
        make_response(response, FirmwareDownload::STATUS_SEQUENCE_ERROR, sequence,
                      expected_sequence_for_response());
        return true;
    }

    if (sequence >= chunk_count) {
        make_response(response, FirmwareDownload::STATUS_SEQUENCE_ERROR, sequence,
                      expected_sequence_for_response());
        return true;
    }

    packet_sequence = sequence;
    clear_packet();
    make_response(response, FirmwareDownload::STATUS_READY, sequence,
                  expected_sequence_for_response());
    return true;
}

bool handle_data(const can_Message_t& message, can_Message_t& response) {
    const uint16_t sequence = static_cast<uint16_t>(packet_sequence);
    if (message.len != 8 || message.buf[0] != FirmwareDownload::FRAME_DATA ||
        message.buf[1] != next_frame_index ||
        next_frame_index >= FirmwareDownload::DATA_FRAME_COUNT) {
        abort_packet(response, FirmwareDownload::STATUS_RETRY_DATA, sequence);
        return true;
    }

    const uint32_t offset = static_cast<uint32_t>(next_frame_index) * 6;
    const uint32_t bytes_to_copy =
        (FirmwareDownload::CHUNK_SIZE - offset < 6)
            ? FirmwareDownload::CHUNK_SIZE - offset
            : 6;
    std::memcpy(&packet_buffer[offset], &message.buf[2], bytes_to_copy);
    if (bytes_to_copy < 6) {
        for (uint32_t i = bytes_to_copy; i < 6; ++i) {
            if (message.buf[2 + i] != 0xFF) {
                abort_packet(response, FirmwareDownload::STATUS_RETRY_DATA, sequence);
                return true;
            }
        }
    }

    ++next_frame_index;
    return true;
}

bool handle_end(const can_Message_t& message, can_Message_t& response) {
    const uint16_t sequence = message.len >= 3 ? read_u16(&message.buf[1]) : 0;
    if (message.len != 7 || message.buf[0] != FirmwareDownload::FRAME_END ||
        sequence != packet_sequence ||
        next_frame_index != FirmwareDownload::DATA_FRAME_COUNT) {
        abort_packet(response, FirmwareDownload::STATUS_RETRY_DATA, sequence);
        return true;
    }

    const uint32_t received_crc = read_u32(&message.buf[3]);
    const uint32_t calculated_crc = crc32(packet_buffer, sizeof(packet_buffer));
    if (received_crc != calculated_crc) {
        abort_packet(response, FirmwareDownload::STATUS_RETRY_CHECKSUM, sequence);
        return true;
    }

    if (duplicate_packet) {
        if (sequence != last_packet_sequence || received_crc != last_packet_crc) {
            abort_packet(response, FirmwareDownload::STATUS_RETRY_CHECKSUM, sequence);
            return true;
        }
    } else {
        if (!write_packet(sequence)) {
            abort_packet(response, FirmwareDownload::STATUS_FLASH_ERROR, sequence);
            return true;
        }
        last_packet_sequence = sequence;
        last_packet_crc = received_crc;
        last_packet_valid = true;
        ++next_sequence;
    }

    receive_state = ReceiveState::waiting_for_start;
    const FirmwareDownload::Status status =
        next_sequence == chunk_count ? FirmwareDownload::STATUS_COMPLETE
                                     : FirmwareDownload::STATUS_ACCEPTED;
    make_response(response, status, sequence, expected_sequence_for_response());
    return true;
}

}

bool FirmwareDownload::handle(const can_Message_t& message, can_Message_t& response) {
    response.len = 0;
    if (!message.isExt || message.id != CAN_ID)
        return false;

    if (receive_state == ReceiveState::receiving_data) {
        if (message.len > 0 && message.buf[0] == FRAME_START)
            return handle_start(message, response);
        if (message.len > 0 && message.buf[0] == FRAME_END)
            return handle_end(message, response);
        return handle_data(message, response);
    }

    if (message.len > 0 && message.buf[0] == FRAME_START)
        return handle_start(message, response);

    if (message.len > 0 && message.buf[0] == FRAME_END) {
        const uint16_t sequence = message.len >= 3 ? read_u16(&message.buf[1]) : 0;
        if (message.len == 7 && last_packet_valid &&
            sequence == last_packet_sequence && read_u32(&message.buf[3]) == last_packet_crc) {
            const FirmwareDownload::Status status =
                next_sequence == chunk_count ? STATUS_COMPLETE : STATUS_ACCEPTED;
            make_response(response, status, sequence, expected_sequence_for_response());
        }
    }
    return true;
}