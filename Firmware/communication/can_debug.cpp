#include "can_simple.hpp"
#include <odrive_main.h>
#include <cstring>
#include "debug.hpp"

void CANSimple::send_odrive_data(uint16_t msg_id, uint8_t type)
{
    int32_t value;
    if (get_debug_value(type, &value)) {
        can_Message_t txmsg;
        txmsg.id = msg_id;
        txmsg.isExt = true;
        txmsg.len = 8;
        txmsg.buf[0] = 0;
        txmsg.buf[1] = 0;
        txmsg.buf[2] = 0;
        txmsg.buf[3] = type;

        uint32_t encoded_value = static_cast<uint32_t>(value);
        txmsg.buf[4] = static_cast<uint8_t>(encoded_value & 0xFF);
        txmsg.buf[5] = static_cast<uint8_t>((encoded_value >> 8) & 0xFF);
        txmsg.buf[6] = static_cast<uint8_t>((encoded_value >> 16) & 0xFF);
        txmsg.buf[7] = static_cast<uint8_t>((encoded_value >> 24) & 0xFF);
        odCAN->write(txmsg);
    }
}

/**
 * 处理 ODrive 调试命令
 * @param msg CAN 消息
 * @note 该函数根据 CAN 消息中的命令类型执行相应的调试操作
 * READ: 读取参数
 * 非 READ : 写入参数
 *
 */
void CANSimple::odrive_debug(can_Message_t& msg)
{

    uint32_t cmd = readDate32(msg, 0);
    uint32_t value = readDate32(msg, 0 + 32);

    switch (cmd) {
        case READ://读取参数
            send_odrive_data(0x321,(uint8_t)value);
        break;
        case ODRV_M0_SPEED://设置电机0速度
            axes[0]->controller_.input_vel_ = value  / 60;  // 进行速度换算
            break;
        case ODRV_M1_SPEED://设置电机1速度
            axes[1]->controller_.input_vel_ = value  / 60;  // 进行速度换算
            break;

        default:
        break;
    }
    return;
}
