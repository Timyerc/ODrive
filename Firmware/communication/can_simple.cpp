#include "can_simple.hpp"
#include <odrive_main.h>
#include <cstring>

#define USE_USER_CAN_CALLBACKS  1   // 使用自定义CAN回调函数
#define ODRIVE_CUR_DEBUG        0   // 开启电流调试功能
#define FILTER_DEPTH            100 // 滤波深度
#define ODRIVE_CAN_TEST         0   // CAN测试功能开关

// 定义静态成员变量
uint32_t CANSimple::alive = 0;


static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

#if USE_USER_CAN_CALLBACKS
uint8_t CANSimple::readDate8(const can_Message_t& msg, uint8_t index) {
    uint8_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true);
    return data;
}

uint16_t CANSimple::readDate16(const can_Message_t& msg, uint8_t index) {
    uint16_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true) << 8;
    data += can_getSignal<uint8_t>(msg, index + 8, 8, true);
    return data;
}

uint32_t CANSimple::readDate32(const can_Message_t& msg, uint8_t index) {
    uint32_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true) << 24;
    data += can_getSignal<uint8_t>(msg, index + 8, 8, true) << 16;
    data += can_getSignal<uint8_t>(msg, index + 16, 8, true) << 8;
    data += can_getSignal<uint8_t>(msg, index + 24, 8, true);
    return data;
}

// 获取电机转速
bool CANSimple::sendMotorSpeed(Axis* axis, uint32_t motorNum) {
    can_Message_t txmsg;
    int16_t Speed = 0;
    uint16_t encoder = 0;

    txmsg.id = motorNum;  // heartbeat ID
    txmsg.isExt = true;

    encoder = (uint16_t)((*axis->controller_.pos_estimate_circular_src_) * 65536.0f);  // 计算位置
    // Speed = ((*axis->controller_.vel_estimate_src_) * 60.0f + 32768.0f);//计算速度
    Speed = (int16_t)((*axis->controller_.vel_estimate_src_) * 60.0f * axis->motor_.config_.pole_pairs);  // 计算速度*极对数

    if (txmsg.id == 0x02 || txmsg.id == 0x11) {
        Speed = -Speed;
    }
    // 状态码
    // if(axis->motor_.user_error_) {
    //     txmsg.len = 5;
    //     txmsg.buf[0] = 0x04;  // 自定义故障
    //     txmsg.buf[1] = axis->motor_.user_error_ >> 24;
    //     txmsg.buf[2] = axis->motor_.user_error_ >> 16;
    //     txmsg.buf[3] = axis->motor_.user_error_ >> 8;
    //     txmsg.buf[4] = axis->motor_.user_error_ ;
    // } else 
    if (axis->motor_.error_) {
        txmsg.len = 5;
        txmsg.buf[0] = 0x08;  // 电机故障
        txmsg.buf[1] = axis->motor_.error_ >> 24;
        txmsg.buf[2] = axis->motor_.error_ >> 16;
        txmsg.buf[3] = axis->motor_.error_ >> 8;
        txmsg.buf[4] = axis->motor_.error_ ;
    } else if (axis->encoder_.error_) {
        txmsg.len = 5;
        txmsg.buf[0] = 0x05;  // 霍尔故障
        txmsg.buf[1] = axis->encoder_.error_ >> 24;
        txmsg.buf[2] = axis->encoder_.error_ >> 16;
        txmsg.buf[3] = axis->encoder_.error_ >> 8;
        txmsg.buf[4] = axis->encoder_.error_ ;
    } else if (axis->sensorless_estimator_.error_) {
        txmsg.len = 5;
        txmsg.buf[0] = 0x03;  // 电机过流
        txmsg.buf[1] = axis->sensorless_estimator_.error_ >> 24;
        txmsg.buf[2] = axis->sensorless_estimator_.error_ >> 16;
        txmsg.buf[3] = axis->sensorless_estimator_.error_ >> 8;
        txmsg.buf[4] = axis->sensorless_estimator_.error_ ;
    } else if (axis->controller_.error_) {
        txmsg.len = 5;
        txmsg.buf[0] = 0x01;  //
        txmsg.buf[1] = axis->controller_.error_ >> 24;
        txmsg.buf[2] = axis->controller_.error_ >> 16;
        txmsg.buf[3] = axis->controller_.error_ >> 8;
        txmsg.buf[4] = axis->controller_.error_ ;
    } else if (axis->error_) {
        txmsg.len = 5;
        txmsg.buf[0] = 0x02;  //
        txmsg.buf[1] = axis->error_ >> 24;
        txmsg.buf[2] = axis->error_ >> 16;
        txmsg.buf[3] = axis->error_ >> 8;
        txmsg.buf[4] = axis->error_ ;
    } else {
        txmsg.len = 5;
        txmsg.buf[0] = 0x09;  // 正常状态
        txmsg.buf[1] = encoder >> 8;
        txmsg.buf[2] = encoder;
        txmsg.buf[3] = Speed >> 8;
        txmsg.buf[4] = Speed;
    }
    odCAN->write(txmsg);  // 返回发送的数据
    return true;
}

typedef struct
{
    uint16_t Countdown;//倒计时
    uint16_t Iq_Num;//滤波计数
    uint16_t index;//滤波索引
    float Iq_Sum;//滤波和
    float Iq_Filter[FILTER_DEPTH];//滤波数组
} Cur_Filter_t;

Cur_Filter_t m0_Cur;
Cur_Filter_t m1_Cur;
Cur_Filter_t ibus;

// 简单移动平均
float MovingAverage(Cur_Filter_t* filter, float new_sample) {
    filter->Iq_Sum -= filter->Iq_Filter[filter->index];
    filter->Iq_Sum += new_sample;
    filter->Iq_Filter[filter->index] = new_sample;
    filter->index = (filter->index + 1) % FILTER_DEPTH;

    return filter->Iq_Sum / FILTER_DEPTH;
}

// 电流过载故障处理
void CANSimple::motor_current_fault(void) {
    axes[0]->controller_.input_vel_ = 0;// 速度清零
    axes[0]->motor_.current_control_.Iq_measured = 0.0f;// 测量电流清零
    axes[1]->controller_.input_vel_ = 0;// 速度清零
    axes[1]->motor_.current_control_.Iq_measured = 0.0f;// 测量电流清零
    safety_critical_disarm_motor_pwm(axes[0]->motor_);// 关闭PWM输出
    safety_critical_disarm_motor_pwm(axes[1]->motor_);// 关闭PWM输出
}

//霍尔状态检测
void CANSimple::hall_error_handing(void) {
    if(axes[0]->encoder_.hall_state_ == 0 || axes[0]->encoder_.hall_state_ == 7) {
        axes[0]->requested_state_ = Axis::AXIS_STATE_IDLE;//进入空闲状态
        axes[0]->motor_.user_error_ |= ERROR_M0_ILLEGAL_HALL;// 设置霍尔错误标志
    }

    if(axes[1]->encoder_.hall_state_ == 0 || axes[1]->encoder_.hall_state_ == 7) {
        axes[1]->requested_state_ = Axis::AXIS_STATE_IDLE;//进入空闲状态
        axes[1]->motor_.user_error_ |= ERROR_M1_ILLEGAL_HALL;// 设置霍尔错误标志
    }
}

//热插拔错误处理
void CANSimple::hot_plugging_error_handing(void) {
    if(axes[0]->motor_.user_error_ & ERROR_M0_ILLEGAL_HALL) {
        // 通过清除错误并重新进入闭环控制尝试恢复
        if(axes[0]->encoder_.hall_state_ != 0 && axes[0]->encoder_.hall_state_ != 7) {
            axes[0]->clear_errors();
            axes[0]->controller_.input_vel_ = 0;
            axes[0]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[0]->motor_.user_error_ = 0;
        }
    }
    if(axes[1]->motor_.user_error_ & ERROR_M1_ILLEGAL_HALL) {
        if(axes[1]->encoder_.hall_state_ != 0 && axes[1]->encoder_.hall_state_ != 7) {
            // 通过清除错误并重新进入闭环控制尝试恢复
            axes[1]->clear_errors();
            axes[1]->controller_.input_vel_ = 0;
            axes[1]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[1]->motor_.user_error_ = 0;
        }
    }
}

int32_t m0_absoluteValue; // 毫安
int32_t m1_absoluteValue; // 毫安
int32_t ibus_absoluteValue; // 毫安
//电流过载保护输出
void CANSimple::motor_overload_output(void) {
    //int32_t m0_absoluteValue = (int32_t)(axes[0]->motor_.current_control_.Ibus * 1000.0f); // 毫安
    //int32_t m1_absoluteValue = (int32_t)(axes[1]->motor_.current_control_.Ibus * 1000.0f); // 毫安
    m0_absoluteValue = (int32_t)(MovingAverage(&m0_Cur,  axes[0]->motor_.current_control_.Ibus) * 1000.0f); // 毫安
    m1_absoluteValue = (int32_t)(MovingAverage(&m1_Cur,  axes[1]->motor_.current_control_.Ibus) * 1000.0f); // 毫安
    ibus_absoluteValue = (int32_t)(MovingAverage(&ibus,  ibus_) * 1000.0f); // 毫安
    if(m0_absoluteValue > axes[0]->config_.current_threshold_mA) {//过流保护
        m0_Cur.Countdown++;
        if(m0_Cur.Countdown >= axes[0]->config_.heartbeat_rate_ms) { //持续3秒以上
            motor_current_fault();//关闭所有电机PWM输出
            axes[0]->motor_.user_error_ |= ERROR_M0_OVER_CURRENT;// 设置用户过流错误标志
        }
    } else {
        m0_Cur.Countdown = 0;// 复位计数
    }

    if(m1_absoluteValue > axes[1]->config_.current_threshold_mA) {//过流保护
        m1_Cur.Countdown++;
        if(m1_Cur.Countdown >= axes[1]->config_.heartbeat_rate_ms) { //持续3秒以上
            motor_current_fault();//关闭所有电机PWM输出
            axes[1]->motor_.user_error_ |= ERROR_M1_OVER_CURRENT;// 设置用户过流错误标志
        }
    } else {
        m1_Cur.Countdown = 0;// 复位计数
    }
#if ODRIVE_CUR_DEBUG
    can_Message_t txmsg;
    txmsg.id = 0x30;
    txmsg.isExt = true;
    txmsg.len = 8;
    if(m1_absoluteValue > axes[1]->config_.current_threshold_mA || m0_absoluteValue > axes[0]->config_.current_threshold_mA) {
        m0_absoluteValue = 4000;
    }
    if(error_) {
        m1_absoluteValue = 0;
        m0_absoluteValue = 0;
    }

#if 1
    txmsg.buf[0] = m0_absoluteValue >> 24;
    txmsg.buf[1] = m0_absoluteValue >> 16;
    txmsg.buf[2] = m0_absoluteValue >> 8;
    txmsg.buf[3] = m0_absoluteValue;

    txmsg.buf[4] = m1_absoluteValue >> 24;
    txmsg.buf[5] = m1_absoluteValue >> 16;
    txmsg.buf[6] = m1_absoluteValue >> 8;
    txmsg.buf[7] = m1_absoluteValue;
#else
    // 使用除法和模运算提取各位
    txmsg.buf[0] = (m0_absoluteValue / 1000) % 10;
    txmsg.buf[1] = (m0_absoluteValue / 100) % 10;
    txmsg.buf[2] = (m0_absoluteValue / 10) % 10;
    txmsg.buf[3] = m0_absoluteValue % 10;

    txmsg.buf[4] = (m1_absoluteValue / 1000) % 10;
    txmsg.buf[5] = (m1_absoluteValue / 100) % 10;
    txmsg.buf[6] = (m1_absoluteValue / 10) % 10;
    txmsg.buf[7] = m1_absoluteValue % 10;
#endif
    odCAN->write(txmsg);
#endif
}

// 保活任务
void CANSimple::keepAlive(Axis* axis) {
    alive++;
    if (alive >= 0x10) {  // 大约1秒没有收到控制指令，停止电机
        axes[0]->controller_.input_vel_ = 0;
        axes[1]->controller_.input_vel_ = 0;
        alive = 0;
    }
    motor_overload_output();//电流过载保护输出
    hall_error_handing();//霍尔状态检测
    hot_plugging_error_handing();//热插拔错误
#if ODRIVE_CAN_TEST
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;

    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = alive >> 24;
    txmsg.buf[1] = alive >> 16;
    txmsg.buf[2] = alive >> 8;
    txmsg.buf[3] = alive ;
    odCAN->write(txmsg);
#endif
}

// 获取电机电流阈值
void CANSimple::get_motor_current_threshold(uint8_t motorNum, uint8_t msg_id)
{
    can_Message_t txmsg;
    txmsg.id = 0x01;
    txmsg.isExt = true;
    txmsg.len = 6;

    txmsg.buf[0] = msg_id;
    txmsg.buf[1] = motorNum;
    if(motorNum == 0x0) {//获取两个电机的电流阈值
        txmsg.buf[2] = axes[0]->config_.heartbeat_rate_ms >> 8;
        txmsg.buf[3] = axes[0]->config_.heartbeat_rate_ms;
        txmsg.buf[4] = axes[0]->config_.current_threshold_mA >> 8;
        txmsg.buf[5] = axes[0]->config_.current_threshold_mA;
    }
    else if(motorNum == 0x1) {//获取单个电机的电流阈值
        txmsg.buf[2] = axes[1]->config_.heartbeat_rate_ms >> 8;
        txmsg.buf[3] = axes[1]->config_.heartbeat_rate_ms;
        txmsg.buf[4] = axes[1]->config_.current_threshold_mA >> 8;
        txmsg.buf[5] = axes[1]->config_.current_threshold_mA;
    }
    odCAN->write(txmsg);
}

// 设置电机电流阈值
void CANSimple::set_motor_current_threshold(uint8_t motorNum, uint8_t msg_id,uint16_t rate_ms, uint16_t current_mA)
{
    if(current_mA < 100 || rate_ms < 1 || motorNum > 1) return; //最小100毫安，最小100ms
    if(motorNum == 0x0) {//设置电机0的电流阈值
        axes[0]->config_.heartbeat_rate_ms = rate_ms;
        axes[0]->config_.current_threshold_mA = current_mA;
    }
    else if(motorNum == 0x1) {//设置电机1的电流阈值
        axes[1]->config_.heartbeat_rate_ms = rate_ms;
        axes[1]->config_.current_threshold_mA = current_mA;
    }
    get_motor_current_threshold(motorNum, msg_id);//返回设置结果
    odrv.save_configuration();
    odrv.reboot();
}

// 获取电机最大速度限制
void CANSimple::get_motor_max_speed_limit(uint8_t msg_id)
{
    can_Message_t txmsg;
    txmsg.id = 0x01;
    txmsg.isExt = true;
    txmsg.len = 5;

    txmsg.buf[0] = msg_id;
    txmsg.buf[1] = axes[0]->config_.max_speed_limit >> 8;
    txmsg.buf[2] = axes[0]->config_.max_speed_limit;
    txmsg.buf[3] = axes[1]->config_.max_speed_limit >> 8;
    txmsg.buf[4] = axes[1]->config_.max_speed_limit;

    odCAN->write(txmsg);
}

// 设置电机速度环pid参数
void CANSimple::set_motor_vel_pid(uint8_t id, uint16_t velGain, uint16_t velIntegratorGain)
{
    if (id == 0) {
        axes[0]->controller_.config_.vel_gain = (float)velGain / 1000;
        axes[0]->controller_.config_.vel_integrator_gain = (float)velIntegratorGain / 1000;
    } else {
        axes[1]->controller_.config_.vel_gain = (float)velGain / 1000;
        axes[1]->controller_.config_.vel_integrator_gain = (float)velIntegratorGain / 1000;
    }
    odrv.save_configuration();
}

// 设置电机最大速度限制
void CANSimple::set_motor_max_speed_limit(uint8_t msg_id, uint16_t m0_max_speed,uint16_t m1_max_speed)
{
    axes[0]->config_.max_speed_limit = m0_max_speed;
    axes[1]->config_.max_speed_limit = m1_max_speed;

    get_motor_max_speed_limit(msg_id);//返回设置结果
    odrv.save_configuration();
    odrv.reboot();
}


void CANSimple::handle_can_message(can_Message_t& msg) {
#if ODRIVE_CAN_TEST
    odCAN->write(msg);//返回接收到的数据
#endif

    if (msg.id == axes[0]->config_.can_node_id || msg.id == axes[1]->config_.can_node_id) {
        axes[0]->watchdog_feed();
        axes[1]->watchdog_feed();
        alive = 0;  // 收到消息，清零保活计数
    }else if(msg.id == 0x123){//debug
        odrive_debug(msg);
        axes[0]->watchdog_feed();
        axes[1]->watchdog_feed();
        alive = 0;  // 收到消息，清零保活计数
    } else {
        return;
    }
    canMessage_t command;
    command.cmd = readDate8(msg, 0);
    switch (command.cmd) {
    case DRIVE_COMMAND0_SPEED:  // 轮子转速设置0X01
        // Odrive转速以秒为单位，默认最大50转每秒，需要做一个转换
        command.leftSpeed = readDate16(msg, 8);
        axes[0]->controller_.input_vel_ = (command.leftSpeed - 32768) * axes[0]->config_.max_speed_limit  / 32768;  // 进行速度换算

        command.rightSpeed = readDate16(msg, 24);
        axes[1]->controller_.input_vel_ = (command.rightSpeed - 32768) * axes[1]->config_.max_speed_limit  / 32768;  // 进行速度换算
        break;

    case DRIVE_COMMAND0_GET_SPEED:  // 速度查询06
        command.SpeedRequst = readDate8(msg, 8);

        if (axes[0]->config_.can_node_id == 0x01) {  // 行走轮
            if (command.SpeedRequst == 0x01) {  // 左轮
                sendMotorSpeed(axes[0], 0x02);
            } else if (command.SpeedRequst == 0x02) {  // 右轮
                sendMotorSpeed(axes[1], 0x03);
            }
        }
        if (axes[0]->config_.can_node_id == 0x10) {  // 毛刷
            if (command.SpeedRequst == 0x01) {            // 左毛刷
                sendMotorSpeed(axes[0], 0x12);
            } else if (command.SpeedRequst == 0x02) {  // 右毛刷
                sendMotorSpeed(axes[1], 0x13);
            }
        }
        break;
    case DRIVE__SET_CURRENT_THRESHOLD:  // 设置电机电流阈值
        set_motor_current_threshold(readDate8(msg, 8),DRIVE__SET_CURRENT_THRESHOLD,readDate16(msg, 16),readDate16(msg, 32));
        break;
    case DRIVE__GET_CURRENT_THRESHOLD:  // 获取电机电流阈值
        get_motor_current_threshold(readDate8(msg, 8),DRIVE__GET_CURRENT_THRESHOLD);
        break;
    case DRIVE__SET_MAX_SPEED_LIMIT:  // 设置电机最大速度限制
        set_motor_max_speed_limit(DRIVE__SET_MAX_SPEED_LIMIT,readDate16(msg, 8),readDate16(msg, 24));
        break;
    case DRIVE__GET_MAX_SPEED_LIMIT:  // 获取电机最大速度限制
        get_motor_max_speed_limit(DRIVE__GET_MAX_SPEED_LIMIT);
        break;
    case DRIVE_CLEAR_ERRORS:  // 异常状态清除并重新进入闭环模式
        if(!readDate16(msg, 16)) {
            clear_errors_callback(axes[0], msg);
            axes[0]->controller_.input_vel_ = 0;
            axes[0]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[0]->motor_.user_error_ = 0;
        }
        else {
            clear_errors_callback(axes[1], msg);
            axes[1]->controller_.input_vel_ = 0;
            axes[1]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[1]->motor_.user_error_ = 0;
        }
        break;
    case DRIVE__SET_VEL_PID:
        set_motor_vel_pid(readDate8(msg, 8), readDate16(msg, 16), readDate16(msg, 32));
        break;
    case DRIVE_RESTART:  // 重新启动
        odrv.reboot();
        break;

    default:
        break;
    }
}
#else

void CANSimple::handle_can_message(can_Message_t& msg) {
    // This functional way of handling the messages is neat and is much cleaner from
    // a data security point of view, but it will require some tweaking to fix the syntax.
    //
    // auto func = callback_map.find(msg.id);
    // if(func != callback_map.end()){
    //     func->second(msg);
    // }

    //     Frame
    // nodeID | CMD
    // 6 bits | 5 bits
    uint32_t nodeID = get_node_id(msg.id);
    uint32_t cmd = get_cmd_id(msg.id);

    Axis* axis = nullptr;

    bool validAxis = false;
    for (uint8_t i = 0; i < AXIS_COUNT; i++) {
        if ((axes[i]->config_.can_node_id == nodeID) && (axes[i]->config_.can_node_id_extended == msg.isExt)) {
            axis = axes[i];
            if (!validAxis) {
                validAxis = true;
            } else {
                // Duplicate can IDs, don't assign to any axis
                odCAN->set_error(ODriveCAN::ERROR_DUPLICATE_CAN_IDS);
                validAxis = false;
                break;
            }
        }
    }

    if (validAxis) {
        axis->watchdog_feed();
        switch (cmd) {
#if 0
        case MSG_CO_NMT_CTRL:
            break;
        case MSG_CO_HEARTBEAT_CMD:
            break;
        case MSG_ODRIVE_HEARTBEAT:
            // We don't currently do anything to respond to ODrive heartbeat messages
            break;
        case MSG_ODRIVE_ESTOP:
            estop_callback(axis, msg);
            break;
        case MSG_GET_MOTOR_ERROR:
            get_motor_error_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ERROR:
            get_encoder_error_callback(axis, msg);
            break;
        case MSG_GET_SENSORLESS_ERROR:
            get_sensorless_error_callback(axis, msg);
            break;
        case MSG_SET_AXIS_NODE_ID:
            set_axis_nodeid_callback(axis, msg);
            break;
        case MSG_SET_AXIS_REQUESTED_STATE:
            set_axis_requested_state_callback(axis, msg);
            break;
        case MSG_SET_AXIS_STARTUP_CONFIG:
            set_axis_startup_config_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_ESTIMATES:
            get_encoder_estimates_callback(axis, msg);
            break;
        case MSG_GET_ENCODER_COUNT:
            get_encoder_count_callback(axis, msg);
            break;
        case MSG_SET_INPUT_POS:
            set_input_pos_callback(axis, msg);
            break;
        case MSG_SET_INPUT_VEL:
            set_input_vel_callback(axis, msg);
            break;
        case MSG_SET_INPUT_TORQUE:
            set_input_torque_callback(axis, msg);
            break;
        case MSG_SET_CONTROLLER_MODES:
            set_controller_modes_callback(axis, msg);
            break;
        case MSG_SET_VEL_LIMIT:
            set_vel_limit_callback(axis, msg);
            break;
        case MSG_START_ANTICOGGING:
            start_anticogging_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_INERTIA:
            set_traj_inertia_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_ACCEL_LIMITS:
            set_traj_accel_limits_callback(axis, msg);
            break;
        case MSG_SET_TRAJ_VEL_LIMIT:
            set_traj_vel_limit_callback(axis, msg);
            break;
        case MSG_GET_IQ:
            get_iq_callback(axis, msg);
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            get_sensorless_estimates_callback(axis, msg);
            break;
        case MSG_RESET_ODRIVE:
            NVIC_SystemReset();
            break;
        case MSG_GET_VBUS_VOLTAGE:
            get_vbus_voltage_callback(axis, msg);
            break;
        case MSG_CLEAR_ERRORS:
            clear_errors_callback(axis, msg);
            break;
#endif
        default:
            break;
        }
    }
}
#endif

void CANSimple::nmt_callback(Axis* axis, can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis* axis, can_Message_t& msg) {
    axis->error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

void CANSimple::get_motor_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->motor_.error_;
        txmsg.buf[1] = axis->motor_.error_ >> 8;
        txmsg.buf[2] = axis->motor_.error_ >> 16;
        txmsg.buf[3] = axis->motor_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.error_;
        txmsg.buf[1] = axis->encoder_.error_ >> 8;
        txmsg.buf[2] = axis->encoder_.error_ >> 16;
        txmsg.buf[3] = axis->encoder_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_error_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->sensorless_estimator_.error_;
        txmsg.buf[1] = axis->sensorless_estimator_.error_ >> 8;
        txmsg.buf[2] = axis->sensorless_estimator_.error_ >> 16;
        txmsg.buf[3] = axis->sensorless_estimator_.error_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_axis_nodeid_callback(Axis* axis, can_Message_t& msg) {
    axis->config_.can_node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis* axis, can_Message_t& msg) {
    axis->requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 16, true));
}
void CANSimple::set_axis_startup_config_callback(Axis* axis, can_Message_t& msg) {
    // Not Implemented
}

void CANSimple::get_encoder_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->encoder_.pos_estimate_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->encoder_.pos_estimate_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->encoder_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->encoder_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_sensorless_estimates_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        // Undefined behaviour!
        // uint32_t floatBytes = *(reinterpret_cast<int32_t*>(&(axis->encoder_.pos_estimate_)));

        uint32_t floatBytes;
        static_assert(sizeof axis->sensorless_estimator_.pll_pos_ == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.pll_pos_, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->sensorless_estimator_.vel_estimate_);
        std::memcpy(&floatBytes, &axis->sensorless_estimator_.vel_estimate_, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_encoder_count_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_ENCODER_COUNT;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        txmsg.buf[0] = axis->encoder_.shadow_count_;
        txmsg.buf[1] = axis->encoder_.shadow_count_ >> 8;
        txmsg.buf[2] = axis->encoder_.shadow_count_ >> 16;
        txmsg.buf[3] = axis->encoder_.shadow_count_ >> 24;

        txmsg.buf[4] = axis->encoder_.count_in_cpr_;
        txmsg.buf[5] = axis->encoder_.count_in_cpr_ >> 8;
        txmsg.buf[6] = axis->encoder_.count_in_cpr_ >> 16;
        txmsg.buf[7] = axis->encoder_.count_in_cpr_ >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::set_input_pos_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_pos_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis->controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis->controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.control_mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis->controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
}

void CANSimple::set_vel_limit_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::start_anticogging_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis* axis, can_Message_t& msg) {
    axis->trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis->trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis* axis, can_Message_t& msg) {
    axis->controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::get_iq_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;
        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_IQ;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof axis->motor_.current_control_.Iq_setpoint == sizeof floatBytes);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_setpoint, sizeof floatBytes);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        static_assert(sizeof floatBytes == sizeof axis->motor_.current_control_.Iq_measured);
        std::memcpy(&floatBytes, &axis->motor_.current_control_.Iq_measured, sizeof floatBytes);
        txmsg.buf[4] = floatBytes;
        txmsg.buf[5] = floatBytes >> 8;
        txmsg.buf[6] = floatBytes >> 16;
        txmsg.buf[7] = floatBytes >> 24;

        odCAN->write(txmsg);
    }
}

void CANSimple::get_vbus_voltage_callback(Axis* axis, can_Message_t& msg) {
    if (msg.rtr) {
        can_Message_t txmsg;

        txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
        txmsg.id += MSG_GET_VBUS_VOLTAGE;
        txmsg.isExt = axis->config_.can_node_id_extended;
        txmsg.len = 8;

        uint32_t floatBytes;
        static_assert(sizeof vbus_voltage == sizeof floatBytes);
        std::memcpy(&floatBytes, &vbus_voltage, sizeof floatBytes);

        // This also works in principle, but I don't have hardware to verify endianness
        // std::memcpy(&txmsg.buf[0], &vbus_voltage, sizeof vbus_voltage);

        txmsg.buf[0] = floatBytes;
        txmsg.buf[1] = floatBytes >> 8;
        txmsg.buf[2] = floatBytes >> 16;
        txmsg.buf[3] = floatBytes >> 24;

        txmsg.buf[4] = 0;
        txmsg.buf[5] = 0;
        txmsg.buf[6] = 0;
        txmsg.buf[7] = 0;

        odCAN->write(txmsg);
    }
}

void CANSimple::clear_errors_callback(Axis* axis, can_Message_t& msg) {
    axis->clear_errors();
}

void CANSimple::send_heartbeat(Axis* axis) {
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;

    // Axis errors in 1st 32-bit value
    txmsg.buf[0] = axis->error_;
    txmsg.buf[1] = axis->error_ >> 8;
    txmsg.buf[2] = axis->error_ >> 16;
    txmsg.buf[3] = axis->error_ >> 24;

    // Current state of axis in 2nd 32-bit value
    txmsg.buf[4] = axis->current_state_;
    txmsg.buf[5] = axis->current_state_ >> 8;
    txmsg.buf[6] = axis->current_state_ >> 16;
    txmsg.buf[7] = axis->current_state_ >> 24;
    odCAN->write(txmsg);
}

uint32_t CANSimple::get_node_id(uint32_t msgID) {
    return (msgID >> NUM_CMD_ID_BITS);  // Upper 6 or more bits
}

uint8_t CANSimple::get_cmd_id(uint32_t msgID) {
    return (msgID & 0x01F);  // Bottom 5 bits
}
