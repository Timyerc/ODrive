#include "can_simple.hpp"
#include <odrive_main.h>
#include <cstring>


#define READ            0
#define ODRV_VBAT       101
#define ODRV_IBUS       102
#define ODRV_M0_IBUS    103
#define ODRV_M1_IBUS    104
#define ODRV_TEMP       105
#define ODRV_M0_TEMP    106
#define ODRV_M1_TEMP    107
#define ODRV_M0_SPEED   108
#define ODRV_M1_SPEED   109
#define ODRV_M0_HALL    110
#define ODRV_M1_HALL    111
#define ODRV_M0_ANGLE   112
#define ODRV_M1_ANGLE   113
#define ODRV_AIBUS      114
#define ODRV_M0_AIBUS   115
#define ODRV_M1_AIBUS   116

#define CUSTOM_0        200
#define CUSTOM_1        201
#define CUSTOM_2        202
#define CUSTOM_3        203
#define CUSTOM_4        204
#define CUSTOM_5        205
#define CUSTOM_6        206
#define CUSTOM_7        207
#define CUSTOM_8        208
#define CUSTOM_9        209

extern int32_t m0_absoluteValue; // 毫安
extern int32_t m1_absoluteValue; // 毫安
extern int32_t ibus_absoluteValue; // 毫安

void CANSimple::send_odrive_data(uint16_t msg_id, uint8_t type)
{
    int32_t value = 0;
    can_Message_t txmsg;
    txmsg.id = msg_id;
    txmsg.isExt = true;
    txmsg.len = 8;
    switch (type) {//读取数据类型
        case ODRV_VBAT:
            value = (int32_t)(vbus_voltage * 1000.0f);// 毫伏
            break;
        case ODRV_IBUS:
            value = (int32_t)(ibus_ * 1000.0f);// 毫安
            break;
        case ODRV_M0_IBUS:
            value = (int32_t)(axes[0]->motor_.current_control_.Ibus* 1000.0f);// 毫安
            break;
        case ODRV_M1_IBUS:
            value = (int32_t)(axes[1]->motor_.current_control_.Ibus* 1000.0f);// 毫安
            break;
        case ODRV_TEMP:
            //value = (int32_t)(temperature_ * 100.0f);// 摄氏度*100
            break;
        case ODRV_M0_TEMP:
            //value = (int32_t)(*axes[0]->temperature_ * 100.0f);// 摄氏度*100
            break;
        case ODRV_M1_TEMP:
            //value = (int32_t)(*axes[1]->temperature_ * 100.0f);// 摄氏度*100
            break;
        case ODRV_M0_SPEED:
            value = (int32_t)(*axes[0]->controller_.vel_estimate_src_ * 60.0f);// 转速 rpm
            break;
        case ODRV_M1_SPEED:
            value = (int32_t)(*axes[1]->controller_.vel_estimate_src_ * 60.0f);// 转速 rpm
            break;  
        case ODRV_M0_HALL:
            value = axes[0]->encoder_.hall_state_;
            break;
        case ODRV_M1_HALL:
            value = axes[1]->encoder_.hall_state_;
            break;
        case ODRV_M0_ANGLE:
            value = (int32_t)((*axes[0]->controller_.pos_estimate_circular_src_) * 360.0f);  // 转子位置
            break;
        case ODRV_M1_ANGLE:
            value = (int32_t)((*axes[1]->controller_.pos_estimate_circular_src_) * 360.0f);  // 转子位置
            break;
        case ODRV_AIBUS:
            value = ibus_absoluteValue;// 平滑滤波，毫安
            break;
        case ODRV_M0_AIBUS:
            value = m0_absoluteValue;// 平滑滤波，毫安
            break;
        case ODRV_M1_AIBUS:
            value = m1_absoluteValue;// 平滑滤波，毫安    
            break;
        case CUSTOM_0:
            value = (int32_t)(axes[0]->motor_.current_meas_.phB * 1000.0f); // ADC value
            break;
        case CUSTOM_1:
            value = (int32_t)(axes[0]->motor_.current_meas_.phC * 1000.0f); // ADC value
            break;
        case CUSTOM_2:
            value = (int32_t)(axes[1]->motor_.current_meas_.phB * 1000.0f); // ADC value
            break;
        case CUSTOM_3:
            value = (int32_t)(axes[1]->motor_.current_meas_.phC * 1000.0f); // ADC value
            break;
        case CUSTOM_4:
            value = CUSTOM_4;
            break;
        case CUSTOM_5:
            value = CUSTOM_5;
            break;
        case CUSTOM_6:
            value = CUSTOM_6;
            break;
        case CUSTOM_7:
            value = CUSTOM_7;
            break;
        case CUSTOM_8:
            value = CUSTOM_8;
            break;
        case CUSTOM_9:
            value = CUSTOM_9;
            break;
        default:
        break;
    }
    txmsg.buf[0] = 0;
    txmsg.buf[1] = 0;
    txmsg.buf[2] = 0;
    txmsg.buf[3] = type;// type

    // txmsg.buf[4] = value >> 24;
    // txmsg.buf[5] = value >> 16;
    // txmsg.buf[6] = value >> 8;
    // txmsg.buf[7] = value;
    txmsg.buf[4] = value;
    txmsg.buf[5] = value >> 8;
    txmsg.buf[6] = value >> 16;
    txmsg.buf[7] = value >> 24;
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
