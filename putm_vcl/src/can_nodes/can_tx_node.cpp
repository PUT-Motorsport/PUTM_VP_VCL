#include "can_nodes/can_tx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanTxNode::CanTxNode() : Node("can_tx_node") 
{
    // 1. Inicjalizacja CAN
    if (!can_tx_amk.Init(putm_vcl::can_interface_amk)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init AMK CAN socket");
    }
    if (!can_tx_common.Init(putm_vcl::can_interface_common)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init Common CAN socket");
    }

    // 2. Subskrypcje Setpoints (ponieważ każdy idzie do innej struktury DBC, rozbijamy templaty dla czytelności)
    amk_front_left_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/front/left/setpoints", 1, std::bind(&CanTxNode::amk_fl_setpoints_callback, this, std::placeholders::_1));
    amk_front_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/front/right/setpoints", 1, std::bind(&CanTxNode::amk_fr_setpoints_callback, this, std::placeholders::_1));
    amk_rear_left_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/left/setpoints", 1, std::bind(&CanTxNode::amk_rl_setpoints_callback, this, std::placeholders::_1));
    amk_rear_right_setpoints_subscriber = this->create_subscription<msg::AmkSetpoints>(
        "amk/rear/right/setpoints", 1, std::bind(&CanTxNode::amk_rr_setpoints_callback, this, std::placeholders::_1));

    // (Tu zostają Twoje subskrypcje dla actual_values1 i 2 bez zmian - używają template jak miałeś)
    // ... rtd_subscriber itp. ...

    lap_timer_subscriber = this->create_subscription<msg::LapTimer>(
        "lap_timer", 1, std::bind(&CanTxNode::lap_timer_callback, this, std::placeholders::_1));

    can_tx_common_timer = this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this));
}

void CanTxNode::rtd_callback(const msg::Rtd msg) { rtd = msg; }

void CanTxNode::lap_timer_callback(const msg::LapTimer msg){
    PUTM_CAN_M_pc_lap_timer_data_t lap_timer = {0}; // struktura DBC
    lap_timer.best_lap = msg.best_lap;
    lap_timer.current_lap = msg.current_lap;
    lap_timer.delta = msg.delta;
    lap_timer.lap_counter = msg.lap_counter;

    if (!can_tx_common.Send(PUTM_CAN_M_PC_LAP_TIMER_DATA_FRAME_ID, lap_timer)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit Lap Timer");
    }
}

// Setpointy dla każdego koła (używają unikalnych struktur wygenerowanych przez skrypt Pythona)
void CanTxNode::amk_fl_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_M_amk_setpoints_fl_t can_msg = {0};
    can_msg.inverter_on = msg.amk_control.inverter_on;
    can_msg.dc_on = msg.amk_control.dc_on;
    can_msg.enable = msg.amk_control.enable;
    can_msg.error_reset = msg.amk_control.error_reset;
    can_msg.target_torque = msg.target_torque;
    can_msg.torque_positive_limit = msg.torque_positive_limit;
    can_msg.torque_negative_limit = msg.torque_negative_limit;
    if (!can_tx_amk.Send(PUTM_CAN_M_AMK_SETPOINTS_FL_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK FL");
}

void CanTxNode::amk_fr_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_M_amk_setpoints_fr_t can_msg = {0};
    can_msg.inverter_on = msg.amk_control.inverter_on;
    can_msg.dc_on = msg.amk_control.dc_on;
    // ... mapowanie pozostałych analogicznie ...
    if (!can_tx_amk.Send(PUTM_CAN_M_AMK_SETPOINTS_FR_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK FR");
}

void CanTxNode::amk_rl_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_M_amk_setpoints_rl_t can_msg = {0};
    can_msg.inverter_on = msg.amk_control.inverter_on;
    can_msg.dc_on = msg.amk_control.dc_on;
    // ... mapowanie pozostałych analogicznie ...
    if (!can_tx_amk.Send(PUTM_CAN_M_AMK_SETPOINTS_RL_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK RL");
}

void CanTxNode::amk_rr_setpoints_callback(const msg::AmkSetpoints& msg) {
    PUTM_CAN_M_amk_setpoints_rr_t can_msg = {0};
    can_msg.inverter_on = msg.amk_control.inverter_on;
    can_msg.dc_on = msg.amk_control.dc_on;
    // ... mapowanie pozostałych analogicznie ...
    if (!can_tx_amk.Send(PUTM_CAN_M_AMK_SETPOINTS_RR_FRAME_ID, can_msg)) RCLCPP_ERROR(this->get_logger(), "Tx Error AMK RR");
}


void CanTxNode::can_tx_common_callback() {
    // 1. Zbieranie do PcMainData
    PUTM_CAN_M_pc_main_data_t pc_main_data = {0};
    pc_main_data.rtd = rtd.state;
    pc_main_data.inverter_ready = inverter_on_rr & inverter_on_rl & inverter_on_fr; // inverter_on_fl
    pc_main_data.vehicle_speed = (wheel_speed_rr + wheel_speed_rl + wheel_speed_fr) / 3;
    pc_main_data.torque_current = (torque_current_rr + torque_current_rl + torque_current_fr) / 3;
    pc_main_data.inverter_error_fr = inverter_error_fr;
    pc_main_data.inverter_error_fl = 0;
    pc_main_data.inverter_error_rl = inverter_error_rl;
    pc_main_data.inverter_error_rr = inverter_error_rr;
    pc_main_data.inverter_on_fr = inverter_on_fr;
    pc_main_data.inverter_on_fl = 0;
    pc_main_data.inverter_on_rr = inverter_on_rr;
    pc_main_data.inverter_on_rl = inverter_on_rl;

    // Wysyłanie PC MAIN DATA (bez try-catch!)
    if (!can_tx_common.Send(PUTM_CAN_M_PC_MAIN_DATA_FRAME_ID, pc_main_data)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to transmit common CAN frames");
    }

    amk_data_limiter_counter--;
    if(amk_data_limiter_counter <= 0){
        amk_data_limiter_counter = amk_data_limiter;
        
        PUTM_CAN_M_amk_temp_data_t amk_temp_data = {0};
        amk_temp_data.inverter_temp_fl = 0;
        amk_temp_data.inverter_temp_fr = inverter_temp_fr;
        amk_temp_data.inverter_temp_rl = inverter_temp_rl;
        amk_temp_data.inverter_temp_rr = inverter_temp_rr;
        amk_temp_data.motor_temp_fl = 0;
        amk_temp_data.motor_temp_fr = motor_temp_fr;
        amk_temp_data.motor_temp_rl = motor_temp_rl;
        amk_temp_data.motor_temp_rr = motor_temp_rr;

        if (!can_tx_common.Send(PUTM_CAN_M_AMK_TEMP_DATA_FRAME_ID, amk_temp_data)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to transmit AmkTempData frames");
        }
    }
}

// ... zostaw resztę z template'ami do odbierania wartości od AMK jak miałeś ...

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}