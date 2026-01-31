#include "can_simple.hpp"
#include <odrive_main.h>
#include <cstring>


#define READ           0
#define ODRV_VBAT      101
#define ODRV_IBUS      102
#define ODRV_M0_IBUS   103
#define ODRV_M1_IBUS   104
#define ODRV_TEMP      105
#define ODRV_M0_TEMP   106
#define ODRV_M1_TEMP   107
#define ODRV_M0_SPEED  108
#define ODRV_M1_SPEED  109
#define ODRV_M0_HALL   110
#define ODRV_M1_HALL   111




void CANSimple::get_odrive_current(uint16_t msg_id)
{
    can_Message_t txmsg;
    txmsg.id = msg_id;
    txmsg.isExt = true;
    txmsg.len = 7;
    int16_t m0_cur = (int16_t)(m0_absoluteValue);
    int16_t m1_cur = (int16_t)(m1_absoluteValue);
    int16_t ibus = (uint16_t)(ibus_absoluteValue);
    // int16_t m0_cur = (int16_t)(axes[0]->motor_.current_control_.Ibus * 1000.0f);
    // int16_t m1_cur = (int16_t)(axes[1]->motor_.current_control_.Ibus * 1000.0f);
    // int16_t ibus = (uint16_t)(ibus_ * 1000.0f);

    txmsg.buf[0] = ODRV_CUR;
    txmsg.buf[1] = ibus >> 8;
    txmsg.buf[2] = ibus;
    txmsg.buf[3] = m0_cur >> 8;
    txmsg.buf[4] = m0_cur;
    txmsg.buf[5] = m1_cur >> 8;
    txmsg.buf[6] = m1_cur;

    odCAN->write(txmsg);
}

void CANSimple::get_odrive_vbat_speed(uint16_t msg_id)
{
    can_Message_t txmsg;
    txmsg.id = msg_id;
    txmsg.isExt = true;
    txmsg.len = 7;
    int16_t vbat = (uint16_t)(vbus_voltage * 1000.0f);

    int16_t m0_speed = (int16_t)((*axes[0]->controller_.vel_estimate_src_) * 60.0f);
    int16_t m1_speed = (int16_t)((*axes[1]->controller_.vel_estimate_src_) * 60.0f);

    txmsg.buf[0] = ODRV_VBAT;
    txmsg.buf[1] = vbat >> 8;
    txmsg.buf[2] = vbat;
    txmsg.buf[3] = m0_speed >> 8;
    txmsg.buf[4] = m0_speed;
    txmsg.buf[5] = m1_speed >> 8;
    txmsg.buf[6] = m1_speed;

    odCAN->write(txmsg);
}
void CANSimple::send_odrive_data(uint16_t msg_id, uint32_t type, uint32_t value)
{
    can_Message_t txmsg;
    txmsg.id = msg_id;
    txmsg.isExt = true;
    txmsg.len = 8;
    switch (type) {//读取数据类型
        case ODRV_VBAT:
            value = (uint32_t)(vbus_voltage * 1000.0f);// 毫伏
            break;
        case ODRV_IBUS:
            value = (uint32_t)(ibus_ * 1000.0f);// 毫安
            break;
        case ODRV_M0_IBUS:
            value = (uint32_t)(axes[0]->motor_.current_control_.Ibus* 1000.0f);// 毫安
            break;
        case ODRV_M1_IBUS:
            value = (uint32_t)(axes[1]->motor_.current_control_.Ibus* 1000.0f);// 毫安
            break;
        case ODRV_TEMP:
            value = (uint32_t)(odrv.get_board_temperature()* 100.0f);// 摄氏度*100
            break;
        case ODRV_M0_TEMP:
            value = (uint32_t)(axes[0]->motor_.get_motor_temperature()* 100.0f);// 摄氏度*100
            break;
        case ODRV_M1_TEMP:
            value = (uint32_t)(axes[1]->motor_.get_motor_temperature()* 100.0f);// 摄氏度*100
            break;
        case ODRV_M0_SPEED:
            value = (uint32_t)((*axes[0]->controller_.vel_estimate_src_) * 60.0f);// 转速 rpm
            break;
        case ODRV_M1_SPEED:
            value = (uint32_t)((*axes[1]->controller_.vel_estimate_src_) * 60.0f);// 转速 rpm
            break;  
        case ODRV_M0_HALL:
            value = (uint32_t)(axes[0]->sensor_.hall_.raw_angle_deg());// 角度0.1度
            break;
        case ODRV_M1_HALL:
            value = (uint32_t)(axes[1]->sensor_.hall_.raw_angle_deg());// 角度0.1度
            break;

        default:
        break;
    }
    txmsg.buf[0] = 0;
    txmsg.buf[1] = 0;
    txmsg.buf[2] = 0;
    txmsg.buf[3] = type;// type

    txmsg.buf[4] = value >> 24;
    txmsg.buf[5] = value >> 16;
    txmsg.buf[6] = value >> 8;
    txmsg.buf[7] = value;

    odCAN->write(txmsg);
}

void CANSimple::odrive_debug(can_Message_t& msg)
{

    uint32_t cmd = readDate32(msg, 0);
    uint32_t value = readDate32(msg, 0 + 32);

    switch (cmd) {
        case READ://读取
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
// 设置电机电流阈值