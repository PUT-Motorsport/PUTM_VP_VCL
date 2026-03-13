#include "can_nodes/can_tx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanTxNode::CanTxNode() : Node("can_tx_node") 
{
    if (!can_tx_amk.Init(putm_vcl::can_interface_amk)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init AMK CAN socket");
    }
    if (!can_tx_common.Init(putm_vcl::can_interface_common)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init Common CAN socket");
    }

    amk_front_left_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/front/left/setpoints", 1, std::bind(&CanTxNode::amk_fl_setpoints_callback, this, std::placeholders::_1));
    amk_front_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/front/right/setpoints", 1, std::bind(&CanTxNode::amk_fr_setpoints_callback, this, std::placeholders::_1));
    amk_rear_left_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/left/setpoints", 1, std::bind(&CanTxNode::amk_rl_setpoints_callback, this, std::placeholders::_1));
    amk_rear_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/right/setpoints", 1, std::bind(&CanTxNode::amk_rr_setpoints_callback, this, std::placeholders::_1));

    lap_timer_subscriber = this->create_subscription<msg::LapTimer>(
        "lap_timer", 1, std::bind(&CanTxNode::lap_timer_callback, this, std::placeholders::_1));

    can_tx_common_timer = this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this));
}

void CanTxNode::rtd_callback(const msg::Rtd msg) { rtd = msg; }

void CanTxNode::lap_timer_callback(const msg::LapTimer msg){
    PUTM_CAN_M_pc_lap_timer_data_t lap_timer = {0}; 
    lap_timer.best_lap_time = msg.best_lap;
    lap_timer.current_lap_time = msg.current_lap;
    lap_timer.delta_time = msg.delta;
    lap_timer.lap_counter = msg.lap_counter;

    if (!can_tx_common.Send(PUTM_CAN_M_PC_LAP_TIMER_DATA_FRAME_ID, lap_timer)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit Lap Timer");
    }
}

void CanTxNode::amk_fl_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_PT_amk_front_left_setpoints1_t can_msg = {0};
    can_msg.amk_b_inverter_on = msg.amk_control.inverter_on;
    can_msg.amk_b_dc_on = msg.amk_control.dc_on;
    can_msg.amk_b_enable = msg.amk_control.enable;
    can_msg.amk_b_error_reset = msg.amk_control.error_reset;
    can_msg.amk_target_velocity = msg.target_torque;
    can_msg.amk_torque_limit_positive = msg.torque_positive_limit;
    can_msg.amk_torque_limit_negative = msg.torque_negative_limit;
    if (!can_tx_amk.Send(PUTM_CAN_PT_AMK_FRONT_LEFT_SETPOINTS1_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK FL");
}

void CanTxNode::amk_fr_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_PT_amk_front_right_setpoints1_t can_msg = {0};
    can_msg.amk_b_inverter_on = msg.amk_control.inverter_on;
    can_msg.amk_b_dc_on = msg.amk_control.dc_on;
    can_msg.amk_b_enable = msg.amk_control.enable;
    can_msg.amk_b_error_reset = msg.amk_control.error_reset;
    can_msg.amk_target_velocity = msg.target_torque;
    can_msg.amk_torque_limit_positive = msg.torque_positive_limit;
    can_msg.amk_torque_limit_negative = msg.torque_negative_limit;
    if (!can_tx_amk.Send(PUTM_CAN_PT_AMK_FRONT_RIGHT_SETPOINTS1_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK FR");
}

void CanTxNode::amk_rl_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_PT_amk_rear_left_setpoints1_t can_msg = {0};
    can_msg.amk_b_inverter_on = msg.amk_control.inverter_on;
    can_msg.amk_b_dc_on = msg.amk_control.dc_on;
    can_msg.amk_b_enable = msg.amk_control.enable;
    can_msg.amk_b_error_reset = msg.amk_control.error_reset;
    can_msg.amk_target_velocity = msg.target_torque;
    can_msg.amk_torque_limit_positive = msg.torque_positive_limit;
    can_msg.amk_torque_limit_negative = msg.torque_negative_limit;
    if (!can_tx_amk.Send(PUTM_CAN_PT_AMK_REAR_LEFT_SETPOINTS1_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK RL");
}

void CanTxNode::amk_rr_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_PT_amk_rear_right_setpoints1_t can_msg = {0};
    can_msg.amk_b_inverter_on = msg.amk_control.inverter_on;
    can_msg.amk_b_dc_on = msg.amk_control.dc_on;
    can_msg.amk_b_enable = msg.amk_control.enable;
    can_msg.amk_b_error_reset = msg.amk_control.error_reset;
    can_msg.amk_target_velocity = msg.target_torque;
    can_msg.amk_torque_limit_positive = msg.torque_positive_limit;
    can_msg.amk_torque_limit_negative = msg.torque_negative_limit;
    if (!can_tx_amk.Send(PUTM_CAN_PT_AMK_REAR_RIGHT_SETPOINTS1_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK RR");
}

void CanTxNode::can_tx_common_callback() {
    PUTM_CAN_M_pc_main_data_t pc_main_data = {0};
    pc_main_data.rtd = rtd.state;
    pc_main_data.inverters_ready = inverter_on_rr & inverter_on_rl & inverter_on_fr; 
    pc_main_data.vehicle_speed = (wheel_speed_rr + wheel_speed_rl + wheel_speed_fr) / 3;
    pc_main_data.torque_current = (torque_current_rr + torque_current_rl + torque_current_fr) / 3;
    pc_main_data.inv_fr_error = inverter_error_fr;
    pc_main_data.inv_fl_error = 0;
    pc_main_data.inv_rl_error = inverter_error_rl;
    pc_main_data.inv_rr_error = inverter_error_rr;
    pc_main_data.inv_fr_status = inverter_on_fr;
    pc_main_data.inv_fl_status = 0;
    pc_main_data.inv_rr_status = inverter_on_rr;
    pc_main_data.inv_rl_status = inverter_on_rl;

    if (!can_tx_common.Send(PUTM_CAN_M_PC_MAIN_DATA_FRAME_ID, pc_main_data)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit common CAN frames");
    }

    amk_data_limiter_counter--;
    if(amk_data_limiter_counter <= 0){
        amk_data_limiter_counter = amk_data_limiter;
        
        PUTM_CAN_M_pc_temperature_data_t temp_data = {0};
        temp_data.front_left_inverter_temperature = 0;
        temp_data.front_right_inverter_temperature = inverter_temp_fr;
        temp_data.rear_left_inverter_temperature = inverter_temp_rl;
        temp_data.rear_right_inverter_temperature = inverter_temp_rr;
        temp_data.front_left_motor_temperature = 0;
        temp_data.front_right_motor_temperature = motor_temp_fr;
        temp_data.rear_left_motor_temperature = motor_temp_rl;
        temp_data.rear_right_motor_temperature = motor_temp_rr;

        if (!can_tx_common.Send(PUTM_CAN_M_PC_TEMPERATURE_DATA_FRAME_ID, temp_data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transmit AmkTempData frames");
        }
    }
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}