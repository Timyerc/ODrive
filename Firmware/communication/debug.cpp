#include "debug.hpp"

#include "interface_uart.h"
#include <odrive_main.h>

#define READ 0
#define ODRV_VBAT 101
#define ODRV_IBUS 102
#define ODRV_M0_IBUS 103
#define ODRV_M1_IBUS 104
#define ODRV_TEMP 105
#define ODRV_M0_TEMP 106
#define ODRV_M1_TEMP 107
#define ODRV_M0_SPEED 108
#define ODRV_M1_SPEED 109
#define ODRV_M0_HALL 110
#define ODRV_M1_HALL 111
#define ODRV_M0_ANGLE 112
#define ODRV_M1_ANGLE 113
#define ODRV_AIBUS 114
#define ODRV_M0_AIBUS 115
#define ODRV_M1_AIBUS 116
#define CUSTOM_0 200
#define CUSTOM_1 201
#define CUSTOM_2 202
#define CUSTOM_3 203
#define CUSTOM_4 204
#define CUSTOM_5 205
#define CUSTOM_6 206
#define CUSTOM_7 207
#define CUSTOM_8 208
#define CUSTOM_9 209

extern int32_t m0_absoluteValue;
extern int32_t m1_absoluteValue;
extern int32_t ibus_absoluteValue;

namespace {

constexpr uint8_t kFrameHeader = 0xFF;
constexpr uint8_t kRequestLength = 0x03;
constexpr uint8_t kPayloadLength = 0x04;
constexpr size_t kRequestFrameSize = 10;

uint8_t candidate[kRequestFrameSize];
size_t candidate_length = 0;

void process_request(const uint8_t* frame);
void process_byte(uint8_t byte, uint8_t* output, size_t output_capacity, size_t* output_length);

bool append_output(uint8_t byte, uint8_t* output, size_t output_capacity, size_t* output_length) {
    if (*output_length >= output_capacity)
        return false;
    output[(*output_length)++] = byte;
    return true;
}

uint32_t read_u32_be(const uint8_t* data) {
    return (static_cast<uint32_t>(data[0]) << 24) |
           (static_cast<uint32_t>(data[1]) << 16) |
           (static_cast<uint32_t>(data[2]) << 8) |
           static_cast<uint32_t>(data[3]);
}

bool get_debug_value(uint8_t code, int32_t* value) {
    switch (code) {
        case ODRV_VBAT:
            *value = static_cast<int32_t>(vbus_voltage * 1000.0f);
            return true;
        case ODRV_IBUS:
            *value = static_cast<int32_t>(ibus_ * 1000.0f);
            return true;
        case ODRV_M0_IBUS:
            if (!axes[0])
                return false;
            *value = static_cast<int32_t>(axes[0]->motor_.current_control_.Ibus * 1000.0f);
            return true;
        case ODRV_M1_IBUS:
            if (!axes[1])
                return false;
            *value = static_cast<int32_t>(axes[1]->motor_.current_control_.Ibus * 1000.0f);
            return true;
        case ODRV_TEMP:
        case ODRV_M0_TEMP:
        case ODRV_M1_TEMP:
            *value = 0;
            return true;
        case ODRV_M0_SPEED:
            if (!axes[0] || !axes[0]->controller_.vel_estimate_src_)
                return false;
            *value = static_cast<int32_t>(*axes[0]->controller_.vel_estimate_src_ * 60.0f);
            return true;
        case ODRV_M1_SPEED:
            if (!axes[1] || !axes[1]->controller_.vel_estimate_src_)
                return false;
            *value = static_cast<int32_t>(*axes[1]->controller_.vel_estimate_src_ * 60.0f);
            return true;
        case ODRV_M0_HALL:
            if (!axes[0])
                return false;
            *value = axes[0]->encoder_.hall_state_;
            return true;
        case ODRV_M1_HALL:
            if (!axes[1])
                return false;
            *value = axes[1]->encoder_.hall_state_;
            return true;
        case ODRV_M0_ANGLE:
            if (!axes[0] || !axes[0]->controller_.pos_estimate_circular_src_)
                return false;
            *value = static_cast<int32_t>(*axes[0]->controller_.pos_estimate_circular_src_ * 3600.0f);
            return true;
        case ODRV_M1_ANGLE:
            if (!axes[1] || !axes[1]->controller_.pos_estimate_circular_src_)
                return false;
            *value = static_cast<int32_t>(*axes[1]->controller_.pos_estimate_circular_src_ * 3600.0f);
            return true;
        case ODRV_AIBUS:
            *value = ibus_absoluteValue;
            return true;
        case ODRV_M0_AIBUS:
            *value = m0_absoluteValue;
            return true;
        case ODRV_M1_AIBUS:
            *value = m1_absoluteValue;
            return true;
        case CUSTOM_0:
            if (!axes[0])
                return false;
            *value = static_cast<int32_t>(axes[0]->motor_.current_meas_.phB * 1000.0f);
            return true;
        case CUSTOM_1:
            if (!axes[0])
                return false;
            *value = static_cast<int32_t>(axes[0]->motor_.current_meas_.phC * 1000.0f);
            return true;
        case CUSTOM_2:
            if (!axes[1])
                return false;
            *value = static_cast<int32_t>(axes[1]->motor_.current_meas_.phB * 1000.0f);
            return true;
        case CUSTOM_3:
            if (!axes[1])
                return false;
            *value = static_cast<int32_t>(axes[1]->motor_.current_meas_.phC * 1000.0f);
            return true;
        case CUSTOM_4:
            *value = static_cast<int32_t>(odrv.debug_var_);
            return true;
        case CUSTOM_5:
            *value = static_cast<int32_t>(odrv.debug_error_);
            return true;
        case CUSTOM_6:
        case CUSTOM_7:
        case CUSTOM_8:
        case CUSTOM_9:
            *value = code;
            return true;
        default:
            return false;
    }
}

void send_response(uint8_t code, int32_t value) {
    if (!uart4_stream_output_ptr)
        return;

    uint32_t encoded_value = static_cast<uint32_t>(value);
    uint8_t response[9] = {
        kFrameHeader,
        kFrameHeader,
        code,
        kPayloadLength,
        static_cast<uint8_t>(encoded_value & 0xFF),
        static_cast<uint8_t>((encoded_value >> 8) & 0xFF),
        static_cast<uint8_t>((encoded_value >> 16) & 0xFF),
        static_cast<uint8_t>((encoded_value >> 24) & 0xFF),
        0
    };
    for (size_t i = 2; i < 8; ++i)
        response[8] ^= response[i];
    uart4_stream_output_ptr->process_bytes(response, sizeof(response), nullptr);
}

void process_request(const uint8_t* frame) {
    uint8_t command = frame[3];
    uint32_t raw_value = read_u32_be(&frame[5]);
    uint8_t response_code;

    if (command != READ || raw_value > 0xFF)
        return;
    response_code = static_cast<uint8_t>(raw_value);

    int32_t value;
    if (get_debug_value(response_code, &value))
        send_response(response_code, value);
}

void flush_candidate(uint8_t* output, size_t output_capacity, size_t* output_length) {
    uint8_t pending[kRequestFrameSize];
    size_t pending_length = candidate_length;
    for (size_t i = 0; i < pending_length; ++i)
        pending[i] = candidate[i];
    candidate_length = 0;

    size_t next_header = pending_length;
    for (size_t i = 1; i + 1 < pending_length; ++i) {
        if (pending[i] == kFrameHeader && pending[i + 1] == kFrameHeader) {
            next_header = i;
            break;
        }
    }
    if (next_header < pending_length) {
        for (size_t i = 0; i < next_header; ++i)
            append_output(pending[i], output, output_capacity, output_length);
        for (size_t i = next_header; i < pending_length; ++i)
            process_byte(pending[i], output, output_capacity, output_length);
    } else if (pending_length && pending[pending_length - 1] == kFrameHeader) {
        for (size_t i = 0; i + 1 < pending_length; ++i)
            append_output(pending[i], output, output_capacity, output_length);
        candidate[0] = kFrameHeader;
        candidate_length = 1;
    } else {
        for (size_t i = 0; i < pending_length; ++i)
            append_output(pending[i], output, output_capacity, output_length);
    }
}

void process_byte(uint8_t byte, uint8_t* output, size_t output_capacity, size_t* output_length) {
    if (candidate_length == 0) {
        if (byte == kFrameHeader)
            candidate[candidate_length++] = byte;
        else
            append_output(byte, output, output_capacity, output_length);
        return;
    }

    if (candidate_length == 1) {
        if (byte == kFrameHeader) {
            candidate[candidate_length++] = byte;
        } else {
            append_output(candidate[0], output, output_capacity, output_length);
            candidate_length = 0;
            process_byte(byte, output, output_capacity, output_length);
        }
        return;
    }

    candidate[candidate_length++] = byte;
    if ((candidate_length == 3 && candidate[2] != kRequestLength) ||
        (candidate_length == 5 && candidate[4] != kPayloadLength)) {
        flush_candidate(output, output_capacity, output_length);
        return;
    }

    if (candidate_length == kRequestFrameSize) {
        uint8_t checksum = 0;
        for (size_t i = 2; i < 9; ++i)
            checksum ^= candidate[i];
        if (checksum == candidate[9]) {
            process_request(candidate);
            candidate_length = 0;
        } else {
            flush_candidate(output, output_capacity, output_length);
        }
    }
}

}  // namespace

size_t debug_process_uart_bytes(const uint8_t* input, size_t length,
                                uint8_t* passthrough, size_t passthrough_capacity) {
    size_t passthrough_length = 0;
    for (size_t i = 0; i < length; ++i)
        process_byte(input[i], passthrough, passthrough_capacity, &passthrough_length);
    return passthrough_length;
}
