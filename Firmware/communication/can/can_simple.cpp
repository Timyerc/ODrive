
#include "can_simple.hpp"

#include <odrive_main.h>
#include <functional>

bool CANSimple::init() {
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        if (!renew_subscription(i)) {
            return false;
        }
    }

    return true;
}

bool CANSimple::renew_subscription(size_t i) {
    Axis& axis = axes[i];

    // TODO: remove these two lines (see comment in header)
    node_ids_[i] = axis.config_.can.node_id;
    extended_node_ids_[i] = axis.config_.can.is_extended;

    MsgIdFilterSpecs filter = {
        .id = {},
        .mask = (uint32_t)(0xffffffff & 0xff)};
        filter.id = (uint32_t)0x01;
    // if (axis.config_.can.is_extended) {
    //     filter.id = (uint32_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    // } else {
    //     filter.id = (uint16_t)(axis.config_.can.node_id << NUM_CMD_ID_BITS);
    // }

    if (subscription_handles_[i]) {
        canbus_->unsubscribe(subscription_handles_[i]);
    }

    return canbus_->subscribe(
        filter, [](void* ctx, const can_Message_t& msg) {
            ((CANSimple*)ctx)->handle_can_message(msg);
        },
        this, &subscription_handles_[i]);
}

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

/*
故障类型：
    0X01：正常，0X02：电机过温，0X03：电机过流，0X04:制动器异常，0X05：霍尔故障，
    0X06：MOS管故障，0X07：电机缺项，0X08：电机堵转，0X30：相电流偏置值异常，0X31：母线电流偏置值异常；
    0x32:电机未连接，0X33:电机过温，0x34:硬件过流，0x35:制动器过流
*/
bool CANSimple::sendLeftMotorError(const Axis& axis) {
    can_Message_t txmsg;
#ifdef USE_CAN_MOTOR  
    txmsg.id = LEFT_DRIVE_CAN_ID;
#elif USE_CAN_BRUSH
    txmsg.id = LEFT_BRUSH_CAN_ID;
#endif    
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    // uint8_t error = 0x01; 

    // can_setSignal(error, 0, 8, true); // error
    // can_setSignal(, 8, 16, true);  // hall change
    // can_setSignal(, 24, 16, true); // speed
    // can_setSignal(, 40, 8, true);  // frames
    // can_setSignal(, 48, 16, true); // current

    return canbus_->send_message(txmsg);
}

bool CANSimple::sendRightMotorError(const Axis& axis) {
    can_Message_t txmsg;
#ifdef USE_CAN_MOTOR  
    txmsg.id = RIGHT_DRIVE_CAN_ID;
#elif USE_CAN_BRUSH
    txmsg.id = RIGHT_BRUSH_CAN_ID;
#endif    
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    // uint8_t error = 0x01; 

    // can_setSignal(error, 0, 8, true); // error
    // can_setSignal(, 8, 16, true);  // hall change
    // can_setSignal(, 24, 16, true); // speed
    // can_setSignal(, 40, 8, true);  // frames
    // can_setSignal(, 48, 16, true); // current

    return canbus_->send_message(txmsg);
}

bool CANSimple::sendLeftMotorSpeed(const Axis& axis) {
    can_Message_t txmsg;
#ifdef USE_CAN_MOTOR  
    txmsg.id = LEFT_DRIVE_CAN_ID;
#elif USE_CAN_BRUSH
    txmsg.id = LEFT_BRUSH_CAN_ID;
#endif    
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    // can_setSignal(MOTOR_HALL_CHANGE_AND_SPEED, 0, 8, true); // cmd 0x09
    // can_setSignal(, 8, 16, true);  // hall change
    // can_setSignal(, 24, 16, true); // speed
    // can_setSignal(, 40, 8, true);  // frames
    // can_setSignal(, 48, 16, true); // current
    return canbus_->send_message(txmsg);
}
//获取左电机转速
bool CANSimple::sendRightMotorSpeed(const Axis& axis) {
    can_Message_t txmsg;
#ifdef USE_CAN_MOTOR  
    txmsg.id = RIGHT_DRIVE_CAN_ID;
#elif USE_CAN_BRUSH
    txmsg.id = RIGHT_BRUSH_CAN_ID;
#endif    
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    // can_setSignal(MOTOR_HALL_CHANGE_AND_SPEED, 0, 8, true); // cmd 0x09
    // can_setSignal(, 8, 16, true);  // hall change
    // can_setSignal(, 24, 16, true); // speed
    // can_setSignal(, 40, 8, true);  // frames
    // can_setSignal(, 48, 16, true); // current
    return canbus_->send_message(txmsg);
}
#if 0
//获取设备电压
bool CANSimple::get_adc_voltage_callback(const Axis& axis, const can_Message_t& msg) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ADC_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
    if (gpio_num < GPIO_COUNT) {
        auto voltage = get_adc_voltage(get_gpio(gpio_num));
        can_setSignal<float>(txmsg, voltage, 0, 32, true);
        return canbus_->send_message(txmsg);
    } else {
        return false;
    }
}
#endif
// //返回调试参数
// void  returnUserPara(const can_Message_t& msg)
// {
//     can_Message_t txmsg;
//     auto voltage = 1;
//     txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
//     txmsg.id += MSG_GET_ADC_VOLTAGE;
//     txmsg.isExt = axis.config_.can.is_extended;
//     txmsg.len = 8;
//     can_setSignal<float>(txmsg, voltage, 0, 32, true);
//     return canbus_->send_message(txmsg);
// }
//Odrive接收到的CAN命令都将在此处解析处理
void CANSimple::handle_can_message(const can_Message_t& msg) {
    do_command(msg);
    canbus_->send_message(msg);//原样返回接收到的数据帧，验证收发正常
}
//按照流程Odrive开机时需要进行自检
//然后进入闭环控制模式
//假定参数合适，即可通过VEL命令控制电机的启停、正反、速度
//速度的查询、以及电机就绪状态的查询
//如何监测电机的状态，以及故障出现时，如何处理

#define USE_USER_CAN
#ifdef USE_USER_CAN
void CANSimple::do_command( const can_Message_t& msg) {
    canMessage_t command;
    command.cmd  = readDate8(msg, 0);
    axes[0].watchdog_feed();
    switch(command.cmd){
        case DRIVE_COMMAND0_SPEED://轮子转速设置0X01
            //Odrive转速以秒为单位，默认最大50转每秒，需要做一个转换
            command.leftSpeed = readDate16(msg, 8);
            axes[0].controller_.input_vel_ = (command.leftSpeed - 32768)*50/32768;//进行速度换算

            command.rightSpeed = readDate16(msg, 24);
            axes[1].controller_.input_vel_ = (command.rightSpeed - 32768)*50/32768;//进行速度换算
            //returnUserPara(msg);
            break; 
        case DRIVE_COMMAND0_MODE:
            // 设置电机模式   暂时不允许更改  默认速度模式
            break;
        case DRIVE_COMMAND0_START_STOP:
            // 驱动器启动与停止    （通过修改两个电机的状态实现）
            // const uint8_t state = can_getSignal<uint8_t>(msg, 8, 8, true);
            // if(state == 1){
            //     // 启动 将两个电机设置为闭环模式
            // }else if(state == 2){
            //     // 停止 将两个电机设置为空闲模式
            // }
            break;
        case DRIVE_COMMAND0_INQUIRY:
            // 轮子在线问询     返回电机的错误码
            // const uint8_t motorId = can_getSignal<uint8_t>(msg, 8, 8, true);
            // if(motorId == 1){
            //     sendLeftMotorError(axis);
            // }else if(motorId == 2){
            //     sendRightMotorError(axis);
            // }
            break;
        case DRIVE_COMMAND0_GET_SPEED:
            // 电机转速反馈    内容以及一些信息还未填入api
            // const uint8_t motorId = can_getSignal<uint8_t>(msg, 8, 8, true);
            // if(motorId == 1){
            //     sendLeftMotorSpeed(axis);
            // }else if(motorId == 2){
            //     sendRightMotorSpeed(axis);
            // }
            break;
        case DRIVE_COMMAND0_PHASE_SEQUENCE:
            // 设置电机相序  暂时不做这个功能    初始配置的时候会绑定相序
            break;
        case DRIVE_COMMAND0_MOTOR_REVERSE:
            // 设置电机转向
            // const uint8_t motorId = can_getSignal<uint8_t>(msg, 8, 8, true);
            // const uint8_t motorDir = can_getSignal<uint8_t>(msg, 16, 8, true);
            // if(motorId == 1){
            //     // 左电机
            // }else if(motorId == 2){
            //     // 右电机
            // }
            break;
        default:

            break;    
    }
}
#else
void CANSimple::do_command(Axis& axis, const can_Message_t& msg) {
    const uint32_t cmd = get_cmd_id(msg.id);
    axis.watchdog_feed();
    switch (cmd) {
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
            if (msg.rtr || msg.len == 0)
                get_motor_error_callback(axis);
            break;
        case MSG_GET_ENCODER_ERROR:
            if (msg.rtr || msg.len == 0)
                get_encoder_error_callback(axis);
            break;
        case MSG_GET_SENSORLESS_ERROR:
            if (msg.rtr || msg.len == 0)
                get_sensorless_error_callback(axis);
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
            if (msg.rtr || msg.len == 0)
                get_encoder_estimates_callback(axis);
            break;
        case MSG_GET_ENCODER_COUNT:
            if (msg.rtr || msg.len == 0)
                get_encoder_count_callback(axis);
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
        case MSG_SET_LIMITS:
            set_limits_callback(axis, msg);
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
            if (msg.rtr || msg.len == 0)
                get_iq_callback(axis);
            break;
        case MSG_GET_SENSORLESS_ESTIMATES:
            if (msg.rtr || msg.len == 0)
                get_sensorless_estimates_callback(axis);
            break;
        case MSG_RESET_ODRIVE:
            NVIC_SystemReset();
            break;
        case MSG_GET_BUS_VOLTAGE_CURRENT:
            if (msg.rtr || msg.len == 0)
                get_bus_voltage_current_callback(axis);
            break;
        case MSG_CLEAR_ERRORS://清除故障
            clear_errors_callback(axis, msg);
            break;
        case MSG_SET_LINEAR_COUNT:
            set_linear_count_callback(axis, msg);
            break;
        case MSG_SET_POS_GAIN://设置位置增益
            set_pos_gain_callback(axis, msg);
            break;
        case MSG_SET_VEL_GAINS://设置速度增益
            set_vel_gains_callback(axis, msg);
            break;
        case MSG_GET_ADC_VOLTAGE://读取电源电压
            get_adc_voltage_callback(axis, msg);
            break;
        case MSG_GET_CONTROLLER_ERROR:
            get_controller_error_callback(axis);
            break;
        default:
            break;
    }
}
#endif

void CANSimple::nmt_callback(const Axis& axis, const can_Message_t& msg) {
    // Not implemented
}

void CANSimple::estop_callback(Axis& axis, const can_Message_t& msg) {
    axis.error_ |= Axis::ERROR_ESTOP_REQUESTED;
}

bool CANSimple::get_motor_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_MOTOR_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.motor_.error_, 0, 64, true);

    return canbus_->send_message(txmsg);
}
//获取编码器错误状态
bool CANSimple::get_encoder_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.encoder_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_SENSORLESS_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.sensorless_estimator_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_controller_error_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_CONTROLLER_ERROR;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.controller_.error_, 0, 32, true);

    return canbus_->send_message(txmsg);
}

void CANSimple::set_axis_nodeid_callback(Axis& axis, const can_Message_t& msg) {
    axis.config_.can.node_id = can_getSignal<uint32_t>(msg, 0, 32, true);
}

void CANSimple::set_axis_requested_state_callback(Axis& axis, const can_Message_t& msg) {
    axis.requested_state_ = static_cast<Axis::AxisState>(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_axis_startup_config_callback(Axis& axis, const can_Message_t& msg) {
    // Not Implemented
}

bool CANSimple::get_encoder_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<float>(txmsg, axis.controller_.pos_estimate_linear_src_.any().value_or(0.0f), 0, 32, true);
    can_setSignal<float>(txmsg, axis.controller_.vel_estimate_src_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_sensorless_estimates_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_SENSORLESS_ESTIMATES;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(axis.sensorless_estimator_.pll_pos_));

    can_setSignal<float>(txmsg, axis.sensorless_estimator_.pll_pos_, 0, 32, true);
    can_setSignal<float>(txmsg, axis.sensorless_estimator_.vel_estimate_.any().value_or(0.0f), 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_encoder_count_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ENCODER_COUNT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal<int32_t>(txmsg, axis.encoder_.shadow_count_, 0, 32, true);
    can_setSignal<int32_t>(txmsg, axis.encoder_.count_in_cpr_, 32, 32, true);
    return canbus_->send_message(txmsg);
}

void CANSimple::set_input_pos_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.set_input_pos_and_steps(can_getSignal<float>(msg, 0, 32, true));
    axis.controller_.input_vel_ = can_getSignal<int16_t>(msg, 32, 16, true, 0.001f, 0);
    axis.controller_.input_torque_ = can_getSignal<int16_t>(msg, 48, 16, true, 0.001f, 0);
    axis.controller_.input_pos_updated();
}

void CANSimple::set_input_vel_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_vel_ = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_input_torque_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.input_torque_ = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_controller_modes_callback(Axis& axis, const can_Message_t& msg) {
    Controller::ControlMode const mode = static_cast<Controller::ControlMode>(can_getSignal<int32_t>(msg, 0, 32, true));
    axis.controller_.config_.control_mode = static_cast<Controller::ControlMode>(mode);
    axis.controller_.config_.input_mode = static_cast<Controller::InputMode>(can_getSignal<int32_t>(msg, 32, 32, true));
    axis.controller_.control_mode_updated();
}

void CANSimple::set_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.motor_.config_.current_lim = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::start_anticogging_callback(const Axis& axis, const can_Message_t& msg) {
    axis.controller_.start_anticogging_calibration();
}

void CANSimple::set_traj_vel_limit_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.vel_limit = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_traj_accel_limits_callback(Axis& axis, const can_Message_t& msg) {
    axis.trap_traj_.config_.accel_limit = can_getSignal<float>(msg, 0, 32, true);
    axis.trap_traj_.config_.decel_limit = can_getSignal<float>(msg, 32, 32, true);
}

void CANSimple::set_traj_inertia_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.inertia = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_linear_count_callback(Axis& axis, const can_Message_t& msg) {
    axis.encoder_.set_linear_count(can_getSignal<int32_t>(msg, 0, 32, true));
}

void CANSimple::set_pos_gain_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.pos_gain = can_getSignal<float>(msg, 0, 32, true);
}

void CANSimple::set_vel_gains_callback(Axis& axis, const can_Message_t& msg) {
    axis.controller_.config_.vel_gain = can_getSignal<float>(msg, 0, 32, true);
    axis.controller_.config_.vel_integrator_gain = can_getSignal<float>(msg, 32, 32, true);
}

bool CANSimple::get_iq_callback(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_IQ;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    std::optional<float2D> Idq_setpoint = axis.motor_.current_control_.Idq_setpoint_;
    if (!Idq_setpoint.has_value()) {
        Idq_setpoint = {0.0f, 0.0f};
    }
    
    static_assert(sizeof(float) == sizeof(Idq_setpoint->second));
    static_assert(sizeof(float) == sizeof(axis.motor_.current_control_.Iq_measured_));
    can_setSignal<float>(txmsg, Idq_setpoint->second, 0, 32, true);
    can_setSignal<float>(txmsg, axis.motor_.current_control_.Iq_measured_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_bus_voltage_current_callback(const Axis& axis) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_BUS_VOLTAGE_CURRENT;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    static_assert(sizeof(float) == sizeof(vbus_voltage));
    static_assert(sizeof(float) == sizeof(ibus_));
    can_setSignal<float>(txmsg, vbus_voltage, 0, 32, true);
    can_setSignal<float>(txmsg, ibus_, 32, 32, true);

    return canbus_->send_message(txmsg);
}

bool CANSimple::get_adc_voltage_callback(const Axis& axis, const can_Message_t& msg) {
    can_Message_t txmsg;

    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_GET_ADC_VOLTAGE;
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    auto gpio_num = can_getSignal<uint8_t>(msg, 0, 8, true);
    if (gpio_num < GPIO_COUNT) {
        auto voltage = get_adc_voltage(get_gpio(gpio_num));
        can_setSignal<float>(txmsg, voltage, 0, 32, true);
        return canbus_->send_message(txmsg);
    } else {
        return false;
    }
}

void CANSimple::clear_errors_callback(Axis& axis, const can_Message_t& msg) {
    odrv.clear_errors();  // TODO: might want to clear axis errors only
}

uint32_t CANSimple::service_stack() {
    uint32_t nextServiceTime = UINT32_MAX;
    uint32_t now = HAL_GetTick();

    // TODO: remove this polling loop and replace with protocol hook
    for (size_t i = 0; i < AXIS_COUNT; ++i) {
        bool node_id_changed = (axes[i].config_.can.node_id != node_ids_[i]) || (axes[i].config_.can.is_extended != extended_node_ids_[i]);
        if (node_id_changed) {
            renew_subscription(i);
        }
    }

    struct periodic {
        const uint32_t& rate;
        uint32_t& last_time;
        bool (CANSimple::* callback)(const Axis& axis);
    };

    for (auto& axis : axes) {
        std::array<periodic, 10> periodics = {{
            {axis.config_.can.heartbeat_rate_ms, axis.can_.last_heartbeat, &CANSimple::send_heartbeat},
            {axis.config_.can.encoder_rate_ms, axis.can_.last_encoder, &CANSimple::get_encoder_estimates_callback},
            {axis.config_.can.motor_error_rate_ms, axis.can_.last_motor_error, &CANSimple::get_motor_error_callback},
            {axis.config_.can.encoder_error_rate_ms, axis.can_.last_encoder_error, &CANSimple::get_encoder_error_callback},
            {axis.config_.can.controller_error_rate_ms, axis.can_.last_controller_error, &CANSimple::get_controller_error_callback},
            {axis.config_.can.sensorless_error_rate_ms, axis.can_.last_sensorless_error, &CANSimple::get_sensorless_error_callback},
            {axis.config_.can.encoder_count_rate_ms, axis.can_.last_encoder_count, &CANSimple::get_encoder_count_callback},
            {axis.config_.can.iq_rate_ms, axis.can_.last_iq, &CANSimple::get_iq_callback},
            {axis.config_.can.sensorless_rate_ms, axis.can_.last_sensorless, &CANSimple::get_sensorless_estimates_callback},
            {axis.config_.can.bus_vi_rate_ms, axis.can_.last_bus_vi, &CANSimple::get_bus_voltage_current_callback},
        }};

        MEASURE_TIME(axis.task_times_.can_heartbeat) {
            for (auto& msg : periodics) {
                if (msg.rate > 0) {
                    if ((now - msg.last_time) >= msg.rate) {
                        if (std::invoke(msg.callback, this, axis)) {
                            msg.last_time = now;
                        }
                    }

                    int nextAxisService = msg.last_time + msg.rate - now;
                    nextServiceTime = std::min(nextServiceTime, static_cast<uint32_t>(std::max(0, nextAxisService)));
                }
            }
        }
    }

    return nextServiceTime;
}

bool CANSimple::send_heartbeat(const Axis& axis) {
    can_Message_t txmsg;
    txmsg.id = axis.config_.can.node_id << NUM_CMD_ID_BITS;
    txmsg.id += MSG_ODRIVE_HEARTBEAT;  // heartbeat ID
    txmsg.isExt = axis.config_.can.is_extended;
    txmsg.len = 8;

    can_setSignal(txmsg, axis.error_, 0, 32, true);
    can_setSignal(txmsg, uint8_t(axis.current_state_), 32, 8, true);

    // Motor flags
    uint8_t motorFlags = axis.motor_.error_ != 0;

    // Encoder flags
    uint8_t encoderFlags = axis.encoder_.error_ != 0;

    // Controller flags
    uint8_t controllerFlags =axis.controller_.error_ != 0;
    uint8_t trajDone = uint8_t(axis.controller_.trajectory_done_) << 7;
    controllerFlags |= trajDone;

    can_setSignal(txmsg, motorFlags, 40, 8, true);
    can_setSignal(txmsg, encoderFlags, 48, 8, true);
    can_setSignal(txmsg, controllerFlags, 56, 8, true);

    return canbus_->send_message(txmsg);
}
