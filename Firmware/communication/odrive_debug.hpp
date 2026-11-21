#ifndef __ODRIVE_DEBUG_HPP
#define __ODRIVE_DEBUG_HPP

#include <stddef.h>
#include <stdint.h>

enum DebugCode {
    READ = 0,
    ODRV_VBAT = 101,
    ODRV_IBUS = 102,
    ODRV_M0_IBUS = 103,
    ODRV_M1_IBUS = 104,
    ODRV_TEMP = 105,
    ODRV_M0_TEMP = 106,
    ODRV_M1_TEMP = 107,
    ODRV_M0_SPEED = 108,
    ODRV_M1_SPEED = 109,
    ODRV_M0_HALL = 110,
    ODRV_M1_HALL = 111,
    ODRV_M0_ANGLE = 112,
    ODRV_M1_ANGLE = 113,
    ODRV_AIBUS = 114,
    ODRV_M0_AIBUS = 115,
    ODRV_M1_AIBUS = 116,
    CUSTOM_0 = 200,
    CUSTOM_1 = 201,
    CUSTOM_2 = 202,
    CUSTOM_3 = 203,
    CUSTOM_4 = 204,
    CUSTOM_5 = 205,
    CUSTOM_6 = 206,
    CUSTOM_7 = 207,
    CUSTOM_8 = 208,
    CUSTOM_9 = 209
};

size_t debug_process_uart_bytes(const uint8_t* input, size_t length,
                                uint8_t* passthrough, size_t passthrough_capacity);

// Shared CAN/UART values; false means unknown code or unavailable source.
bool get_debug_value(uint8_t code, int32_t* value);

#endif
