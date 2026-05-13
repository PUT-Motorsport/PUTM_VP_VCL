#include "can_nodes/can_tx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanTxNode::CanTxNode() : Node("can_tx_node") {
    // 1. Inicjalizacja CAN
    if (!can_tx_amk.Init(can_interface_amk)) RCLCPP_ERROR(this->get_logger(), "Init AMK TX failed");
    if (!can_tx_common.Init(can_interface_common)) RCLCPP_ERROR(this->get_logger(), "Init Common TX failed");
    
    rclcpp::QoS qos(1);
    qos.best_effort();
    qos.durability_volatile();
    // 2. Subskrypcje
    amk_front_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/front/right/setpoints", qos, std::bind(&CanTxNode::amk_fr_setpoints_callback, this, std::placeholders::_1));
    amk_rear_left_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/left/setpoints", qos, std::bind(&CanTxNode::amk_rl_setpoints_callback, this, std::placeholders::_1));
    amk_rear_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/right/setpoints", qos, std::bind(&CanTxNode::amk_rr_setpoints_callback, this, std::placeholders::_1));

    amk_front_right_actual_values1_subscriber = this->create_subscription<msg::AmkActualValues1>(
        "amk/front/right/actual_values1", qos, std::bind(&CanTxNode::amk_fr_actual1_callback, this, std::placeholders::_1));
    amk_rear_left_actual_values1_subscriber = this->create_subscription<msg::AmkActualValues1>(
        "amk/rear/left/actual_values1", qos, std::bind(&CanTxNode::amk_rl_actual1_callback, this, std::placeholders::_1));
    amk_rear_right_actual_values1_subscriber = this->create_subscription<msg::AmkActualValues1>(
        "amk/rear/right/actual_values1", qos, std::bind(&CanTxNode::amk_rr_actual1_callback, this, std::placeholders::_1));

    amk_front_right_actual_values2_subscriber = this->create_subscription<msg::AmkActualValues2>(
        "amk/front/right/actual_values2", qos, std::bind(&CanTxNode::amk_fr_actual2_callback, this, std::placeholders::_1));
    amk_rear_left_actual_values2_subscriber = this->create_subscription<msg::AmkActualValues2>(
        "amk/rear/left/actual_values2", qos, std::bind(&CanTxNode::amk_rl_actual2_callback, this, std::placeholders::_1));
    amk_rear_right_actual_values2_subscriber = this->create_subscription<msg::AmkActualValues2>(
        "amk/rear/right/actual_values2", qos, std::bind(&CanTxNode::amk_rr_actual2_callback, this, std::placeholders::_1));

    rtd_subscriber = this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&CanTxNode::rtd_callback, this, std::placeholders::_1));
    lap_timer_subscriber = this->create_subscription<msg::LapTimer>("lap_timer", 1, std::bind(&CanTxNode::lap_timer_callback, this, std::placeholders::_1));
    can_tx_common_timer = this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this));
}

void CanTxNode::rtd_callback(const msg::Rtd msg) { rtd = msg; }

void CanTxNode::lap_timer_callback(const msg::LapTimer msg) {
    PUTM_CAN_M_pc_lap_timer_data_t out{};
    out.best_lap_time = msg.best_lap;
    out.current_lap_time = msg.current_lap;
    out.delta_time = msg.delta;
    out.lap_counter = msg.lap_counter;

    if (!can_tx_common.Send(PUTM_CAN_M_PC_LAP_TIMER_DATA_FRAME_ID, out)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit Lap Timer");
    }
}

// ==== CALLBACKI SETPOINTÓW (WYSYŁANIE) ====
void CanTxNode::amk_fr_setpoints_callback(const msg::AmkSetpoints msg) {
    PUTM_CAN_PT_amk_front_right_setpoints1_t out{};
    out.amk_b_inverter_on = msg.amk_control.inverter_on;
    out.amk_b_dc_on = msg.amk_control.dc_on;
    out.amk_b_enable = msg.amk_control.enable;
    out.amk_b_error_reset = msg.amk_control.error_reset;
    out.amk_target_velocity = msg.target_torque; 
    out.amk_torque_limit_positive = msg.torque_positive_limit;
    out.amk_torque_limit_negative = msg.torque_negative_limit;
    can_tx_amk.Send(PUTM_CAN_PT_AMK_FRONT_RIGHT_SETPOINTS1_FRAME_ID, out);
}

void CanTxNode::amk_rl_setpoints_callback(const msg::AmkSetpoints msg) {
    PUTM_CAN_PT_amk_rear_left_setpoints1_t out{};
    out.amk_b_inverter_on = msg.amk_control.inverter_on;
    out.amk_b_dc_on = msg.amk_control.dc_on;
    out.amk_b_enable = msg.amk_control.enable;
    out.amk_b_error_reset = msg.amk_control.error_reset;
    out.amk_target_velocity = msg.target_torque;
    out.amk_torque_limit_positive = msg.torque_positive_limit;
    out.amk_torque_limit_negative = msg.torque_negative_limit;
    can_tx_amk.Send(PUTM_CAN_PT_AMK_REAR_LEFT_SETPOINTS1_FRAME_ID, out);
}

void CanTxNode::amk_rr_setpoints_callback(const msg::AmkSetpoints msg) {
    PUTM_CAN_PT_amk_rear_right_setpoints1_t out{};
    out.amk_b_inverter_on = msg.amk_control.inverter_on;
    out.amk_b_dc_on = msg.amk_control.dc_on;
    out.amk_b_enable = msg.amk_control.enable;
    out.amk_b_error_reset = msg.amk_control.error_reset;
    out.amk_target_velocity = msg.target_torque;
    out.amk_torque_limit_positive = msg.torque_positive_limit;
    out.amk_torque_limit_negative = msg.torque_negative_limit;
    can_tx_amk.Send(PUTM_CAN_PT_AMK_REAR_RIGHT_SETPOINTS1_FRAME_ID, out);
}

// ==== CALLBACKI DO AGREGACJI ACTUAL VALUES ====
void CanTxNode::amk_fr_actual1_callback(const msg::AmkActualValues1 msg) {
    torque_current_fr = msg.torque_current;
    inverter_ready_fr = msg.amk_status.system_ready;
    inverter_on_fr = msg.amk_status.inverter_on;
    inverter_error_fr = msg.amk_status.error;
    wheel_speed_fr = msg.actual_velocity;
}

void CanTxNode::amk_rl_actual1_callback(const msg::AmkActualValues1 msg) {
    torque_current_rl = msg.torque_current;
    inverter_ready_rl = msg.amk_status.system_ready;
    inverter_on_rl = msg.amk_status.inverter_on;
    inverter_error_rl = msg.amk_status.error;
    wheel_speed_rl = msg.actual_velocity;
}

void CanTxNode::amk_rr_actual1_callback(const msg::AmkActualValues1 msg) {
    torque_current_rr = msg.torque_current;
    inverter_ready_rr = msg.amk_status.system_ready;
    inverter_on_rr = msg.amk_status.inverter_on;
    inverter_error_rr = msg.amk_status.error;
    wheel_speed_rr = msg.actual_velocity;
}

void CanTxNode::amk_fr_actual2_callback(const msg::AmkActualValues2 msg) {
    inverter_temp_fr = abs(msg.temp_inverter) / 10;
    motor_temp_fr = abs(msg.temp_motor) / 10;
}

void CanTxNode::amk_rl_actual2_callback(const msg::AmkActualValues2 msg) {
    inverter_temp_rl = abs(msg.temp_inverter) / 10;
    motor_temp_rl = abs(msg.temp_motor) / 10;
}

void CanTxNode::amk_rr_actual2_callback(const msg::AmkActualValues2 msg) {
    inverter_temp_rr = abs(msg.temp_inverter) / 10;
    motor_temp_rr = abs(msg.temp_motor) / 10;
}

// ==== TIMER GŁÓWNY WYSYŁAJĄCY AGREGOWANE DANE ====
void CanTxNode::can_tx_common_callback() {
    PUTM_CAN_M_pc_main_data_t out{};
    out.rtd = rtd.state;
    out.inverters_ready = inverter_on_rr & inverter_on_rl & inverter_on_fr;
    out.vehicle_speed = (wheel_speed_rr + wheel_speed_rl + wheel_speed_fr) / 3;
    out.torque_current = (torque_current_rr + torque_current_rl + torque_current_fr) / 3;
    
    out.inv_fr_error = inverter_error_fr;
    out.inv_fl_error = 0;
    out.inv_rl_error = inverter_error_rl;
    out.inv_rr_error = inverter_error_rr;

    out.inv_fr_status = inverter_on_fr;
    out.inv_fl_status = 0;
    out.inv_rl_status = inverter_on_rl;
    out.inv_rr_status = inverter_on_rr;

    if (!can_tx_common.Send(PUTM_CAN_M_PC_MAIN_DATA_FRAME_ID, out)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit common CAN frames");
    }

    amk_data_limiter_counter--;
    if (amk_data_limiter_counter == 0) {
        amk_data_limiter_counter = amk_data_limiter;
        
        PUTM_CAN_M_pc_temperature_data_t temp_out{};
        temp_out.front_left_inverter_temperature = 0;
        temp_out.front_right_inverter_temperature = inverter_temp_fr;
        temp_out.rear_left_inverter_temperature = inverter_temp_rl;
        temp_out.rear_right_inverter_temperature = inverter_temp_rr;

        temp_out.front_left_motor_temperature = 0;
        temp_out.front_right_motor_temperature = motor_temp_fr;
        temp_out.rear_left_motor_temperature = motor_temp_rl;
        temp_out.rear_right_motor_temperature = motor_temp_rr;

        if (!can_tx_common.Send(PUTM_CAN_M_PC_TEMPERATURE_DATA_FRAME_ID, temp_out)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transmit AmkTempData frames");
        }
    }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}