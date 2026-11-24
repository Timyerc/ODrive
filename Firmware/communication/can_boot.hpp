#ifndef FIRMWARE_CAN_BOOT_HPP
#define FIRMWARE_CAN_BOOT_HPP

#include "communication/can_helpers.hpp"

namespace FirmwareOTA {

bool handle(const can_Message_t& message, can_Message_t& response);
bool reset_after_response();

}

#endif
