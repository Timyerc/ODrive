#ifndef __DEBUG_HPP
#define __DEBUG_HPP

#include <stddef.h>
#include <stdint.h>

size_t debug_process_uart_bytes(const uint8_t* input, size_t length,
                                uint8_t* passthrough, size_t passthrough_capacity);

#endif
