
#include "can_simple.hpp"
#include <odrive_main.h>

#include <cstring>
static constexpr uint8_t NUM_NODE_ID_BITS = 6;
static constexpr uint8_t NUM_CMD_ID_BITS = 11 - NUM_NODE_ID_BITS;

#define USE_USER_CAN_CALLBACKS 1
#if USE_USER_CAN_CALLBACKS
uint8_t CANSimple::readDate8(const can_Message_t& msg, uint8_t index){
    uint8_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true);
    return data;
}

uint16_t CANSimple::readDate16(const can_Message_t& msg, uint8_t index){
    uint16_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true) << 8;
    data += can_getSignal<uint8_t>(msg, index + 8, 8, true);
    return data;
}

uint32_t CANSimple::readDate32(const can_Message_t& msg, uint8_t index){
    uint32_t data = 0;
    data = can_getSignal<uint8_t>(msg, index, 8, true) << 24;
    data += can_getSignal<uint8_t>(msg, index + 8, 8, true) << 16;
    data += can_getSignal<uint8_t>(msg, index + 16, 8, true) << 8;
    data += can_getSignal<uint8_t>(msg, index + 24, 8, true);
    return data;
}
//获取电机转速
bool CANSimple::sendMotorSpeed(Axis* axis,uint32_t motorNum) {
    can_Message_t txmsg;
    int16_t Speed = 0;
    uint16_t encoder = 0;

    txmsg.id = motorNum;  // heartbeat ID

    txmsg.isExt = true;
    txmsg.len = 5;
    encoder = (*axis->controller_.pos_estimate_circular_src_) * 65536.0f;//计算位置
    //Speed = ((*axis->controller_.vel_estimate_src_) * 60.0f + 32768.0f);//计算速度
    Speed = ((*axis->controller_.vel_estimate_src_) * 60.0f * axis->motor_.config_.pole_pairs);//计算速度*极对数

    if (txmsg.id == 0x02 || txmsg.id == 0x11) {
        Speed = -Speed;
    }

    //状态码
    if(axis->motor_.error_){
    txmsg.buf[0] = 0x08;//电机故障
    }else if(axis->encoder_.error_){
    txmsg.buf[0] = 0x05;//霍尔故障
    }else if(axis->sensorless_estimator_.error_){
    txmsg.buf[0] = 0x03;//电机过流
    }else{
        txmsg.buf[0] = 0x09;//正常状态
    }
    txmsg.buf[1] = encoder >> 8;
    txmsg.buf[2] = encoder;
    txmsg.buf[3] = Speed >> 8;
    txmsg.buf[4] = Speed;
    odCAN->write(txmsg);//返回发送的数据
    return true;
}

uint32_t num = 0;
void CANSimple::keepAlive(Axis* axis) {
    //保活任务
    num++;
    if(num>=0x10){//1秒没有收到控制指令，停止电机
        axes[0]->controller_.input_vel_ = 0;
        axes[1]->controller_.input_vel_ = 0;
        num = 0;
        //停止电机
    }
    can_Message_t txmsg;
    txmsg.id = axis->config_.can_node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis->config_.can_node_id_extended;
    txmsg.len = 8;

    // // 发送保活消息
    // txmsg.buf[0] = num;
    // txmsg.buf[1] = num >> 8;
    // txmsg.buf[2] = num >> 16;
    // txmsg.buf[3] = num >> 24;
    // odCAN->write(txmsg);
}


void CANSimple::handle_can_message(can_Message_t& msg) {
    num = 0;//收到消息，清零保活计数
    if(msg.id == axes[0]->config_.can_node_id || msg.id == axes[1]->config_.can_node_id){
        axes[0]->watchdog_feed();
        axes[1]->watchdog_feed();
    }
    else{
        return;
    }
    canMessage_t command;
    command.cmd  = readDate8(msg, 0);
    switch(command.cmd){
        case DRIVE_COMMAND0_SPEED://轮子转速设置0X01
            //Odrive转速以秒为单位，默认最大50转每秒，需要做一个转换
            command.leftSpeed = readDate16(msg, 8);
            axes[0]->controller_.input_vel_ = (command.leftSpeed - 32768) * axes[0]->controller_.config_.vel_limit/32768;//进行速度换算

            command.rightSpeed = readDate16(msg, 24);
            axes[1]->controller_.input_vel_ = (command.rightSpeed - 32768) * axes[0]->controller_.config_.vel_limit/32768;//进行速度换算
            break; 

        case DRIVE_COMMAND0_GET_SPEED://速度查询
            command.SpeedRequst = readDate8(msg, 8);
            
            if (axes[0], axes[0]->config_.can_node_id < 0x10 && command.SpeedRequst == 0x01) {
                sendMotorSpeed(axes[0],axes[0]->config_.can_node_id + 1);
            }
            else{
                sendMotorSpeed(axes[0],axes[0]->config_.can_node_id + 2);
            }
            if (axes[0], axes[0]->config_.can_node_id < 0x10 && command.SpeedRequst == 0x02) {
                sendMotorSpeed(axes[1],axes[1]->config_.can_node_id + 2);
            }
            else if (command.SpeedRequst == 0x02) {
                sendMotorSpeed(axes[1],axes[1]->config_.can_node_id + 3);
            }
            break;

        case DRIVE_CLEAR_ERRORS://异常状态清除
            clear_errors_callback(axes[0], msg);
            clear_errors_callback(axes[1], msg);
            axes[0]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[1]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            break;
        case DRIVE_MOTOR_CALIBRATION://电机自检
            axes[0]->requested_state_ = Axis::AXIS_STATE_MOTOR_CALIBRATION;
            axes[1]->requested_state_ = Axis::AXIS_STATE_MOTOR_CALIBRATION;
            break;
        case DRIVE_ENCODER_OFFSET_CALIBRATION://编码器校准
            axes[0]->requested_state_ = Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            axes[1]->requested_state_ = Axis::AXIS_STATE_ENCODER_OFFSET_CALIBRATION;
            break;

        case DRIVE_CLOSED_LOOP_CONTROL://进入闭环模式
            axes[0]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            axes[1]->requested_state_ = Axis::AXIS_STATE_CLOSED_LOOP_CONTROL;
            break;
        case DRIVE_IDLE_MODE://空闲模式
            axes[0]->requested_state_ = Axis::AXIS_STATE_IDLE;
            axes[1]->requested_state_ = Axis::AXIS_STATE_IDLE;
            break;
        default:
            break;
    }

            
    //odCAN->write(msg);//返回发送的数据
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

