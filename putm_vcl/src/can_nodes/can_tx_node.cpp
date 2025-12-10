#include "can_nodes/can_tx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using namespace std::chrono;

using std::placeholders::_1;

CanTxNode::CanTxNode()
    : Node("can_tx_node"),
      can_tx_amk(can_interface_amk),
      can_tx_common(can_interface_common),

      // amk_front_left_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
      //     "amk/front/left/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkFrontLeftSetpoints>, this, _1))),
      // amk_front_right_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
      //     "amk/front/right/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkFrontRightSetpoints>, this, _1))),
      amk_rear_left_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
          "amk/rear/left/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkRearLeftSetpoints>, this, _1))),
      // amk_rear_right_setpoints_subscriber(this->create_subscription<msg::AmkSetpoints>(
      //     "amk/rear/right/setpoints", 1, std::bind(&CanTxNode::amk_setpoints_callback<AmkRearRightSetpoints>, this, _1))),

      // amk_front_left_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
      //      "amk/front/left/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkFrontLeftActualValues1>, this, _1))),
      // amk_front_right_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
      //      "amk/front/right/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkFrontRightActualValues1>, this, _1))),
      amk_rear_left_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
           "amk/rear/left/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkRearLeftActualValues1>, this, _1))),
      // amk_rear_right_actual_values1_subscriber(this->create_subscription<msg::AmkActualValues1>(
      //      "amk/rear/right/actual_values1", 1, std::bind(&CanTxNode::amk_actual_values1_callback<AmkRearRightActualValues1>, this, _1))),

      // amk_front_left_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
      //      "amk/front/left/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkFrontLeftActualValues2>, this, _1))),
      //  amk_front_right_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
      //      "amk/front/right/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkFrontRightActualValues2>, this, _1))),
       amk_rear_left_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
           "amk/rear/left/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkRearLeftActualValues2>, this, _1))),
      //  amk_rear_right_actual_values2_subscriber(this->create_subscription<msg::AmkActualValues2>(
      //      "amk/rear/right/actual_values2", 1, std::bind(&CanTxNode::amk_actual_values2_callback<AmkRearRightActualValues2>, this, _1))),

      rtd_subscriber(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&CanTxNode::rtd_callback, this, _1))),

      lap_timer_subscriber(this->create_subscription<msg::LapTimer>(
        "lap_timer", 1, std::bind(&CanTxNode::lap_timer_callback, this, _1))),

      can_tx_common_timer(this->create_wall_timer(10ms, std::bind(&CanTxNode::can_tx_common_callback, this))) {}

void CanTxNode::rtd_callback(const msg::Rtd msg) { rtd = msg; }

void CanTxNode::lap_timer_callback(const msg::LapTimer msg){
  // RCLCPP_INFO(this->get_logger(), "laptimer_callback rclcpp info");
  LapTimer lap_timer;
  lap_timer.best_lap = msg.best_lap;
  lap_timer.current_lap = msg.current_lap;
  lap_timer.delta = msg.delta;
  lap_timer.lap_counter = msg.lap_counter;
  try {
    can_tx_common.transmit(lap_timer);
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to transmit Lap Timer: %s", e.what());
  }
}

template <typename T>
void CanTxNode::amk_setpoints_callback(const msg::AmkSetpoints msg) {
  T amk_setpoints;
  amk_setpoints.amk_control.inverter_on = msg.amk_control.inverter_on;
  amk_setpoints.amk_control.dc_on = msg.amk_control.dc_on;
  amk_setpoints.amk_control.enable = msg.amk_control.enable;
  amk_setpoints.amk_control.error_reset = msg.amk_control.error_reset;
  amk_setpoints.target_torque = msg.target_torque;
  amk_setpoints.torque_positive_limit = msg.torque_positive_limit;
  amk_setpoints.torque_negative_limit = msg.torque_negative_limit;
  try {
     can_tx_amk.transmit(amk_setpoints);
   } catch (const std::runtime_error& e) {
     RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK setpoints: %s", e.what());
   }
}

template <typename T>
void CanTxNode::amk_actual_values1_callback(const msg::AmkActualValues1 msg) {
  // T amk_actual_values1;
  // amk_actual_values1.amk_status.system_ready = msg.amk_status.system_ready;
  // amk_actual_values1.amk_status.error = msg.amk_status.error;
  // amk_actual_values1.amk_status.warn = msg.amk_status.warn;
  // amk_actual_values1.amk_status.quit_dc_on = msg.amk_status.quit_dc_on;
  // amk_actual_values1.amk_status.dc_on = msg.amk_status.dc_on;
  // amk_actual_values1.amk_status.quit_inverter_on = msg.amk_status.quit_inverter_on;
  // amk_actual_values1.amk_status.inverter_on = msg.amk_status.inverter_on;
  // amk_actual_values1.amk_status.derating = msg.amk_status.derating;
  // amk_actual_values1.actual_velocity = msg.actual_velocity;
  // amk_actual_values1.torque_current = msg.torque_current;
  // amk_actual_values1.magnetizing_current = msg.magnetizing_current;
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN24AmkRearLeftActualValues1E") == 0){
    torque_current_rl = msg.torque_current;
    inverter_on_rl = msg.amk_status.inverter_on;
    inverter_error_rl = msg.amk_status.error;
    inverter_ready_rl = msg.amk_status.system_ready;
    wheel_speed_rl = msg.actual_velocity;
  }
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN26AmkFrontRightActualValues1E") == 0){
    torque_current_fr = msg.torque_current;
    inverter_ready_fr = msg.amk_status.system_ready;
    inverter_on_fr = msg.amk_status.inverter_on;
    inverter_error_fr = msg.amk_status.error;
    wheel_speed_fr = msg.actual_velocity;
  }
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN25AmkRearRightActualValues1E") == 0){
    torque_current_rr = msg.torque_current;
    inverter_ready_rr = msg.amk_status.system_ready;
    inverter_on_rr = msg.amk_status.inverter_on;
    inverter_error_rr = msg.amk_status.error;
    wheel_speed_rr = msg.actual_velocity;
  }
  // if(std::strcmp(typeid(T).name(),"N8PUTM_CAN25AmkFrontLeftActualValues1E") == 0){
  //   torque_current_fl = msg.torque_current;
  //   inverter_ready_fl = msg.amk_status.system_ready;
  //   inverter_on_fl = msg.amk_status.inverter_on;
  //   inverter_error_fl = msg.amk_status.error;
  //   wheel_speed_fl = msg.actual_velocity;
  // }
  


  //  try {
  //    can_tx_common.transmit(amk_actual_values1);
  //  } catch (const std::runtime_error& e) {
  //    RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK actual values 1: %s", e.what());
  //  }
}

template <typename T>
void CanTxNode::amk_actual_values2_callback(const msg::AmkActualValues2 msg) {
  // T amk_actual_values2;
  // amk_actual_values2.temp_motor = msg.temp_motor;
  // amk_actual_values2.temp_inverter = msg.temp_inverter;
  // amk_actual_values2.error_info = msg.error_info;
  // amk_actual_values2.temp_igbt = msg.temp_igbt;

  // if(std::strcmp(typeid(T).name(),"N8PUTM_CAN25AmkFrontLeftActualValues2E") == 0){
  //   inverter_temp_fl = abs(msg.temp_inverter)/10;
  //   motor_temp_fl = abs( msg.temp_motor)/10;
  // }
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN26AmkFrontRightActualValues2E") == 0){
    inverter_temp_fr = abs(msg.temp_inverter)/10;
    motor_temp_fr = abs(msg.temp_motor)/10;
  }
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN24AmkRearLeftActualValues2E") == 0){
    inverter_temp_rl = abs(msg.temp_inverter)/10;
    motor_temp_rl =abs(msg.temp_motor)/10;
  }
  if(std::strcmp(typeid(T).name(),"N8PUTM_CAN25AmkRearRightActualValues2E") == 0){
    inverter_temp_rr = abs(msg.temp_inverter)/10;
    motor_temp_rr = abs(msg.temp_motor)/10;
  }

  //  try {
  //   can_tx_common.transmit(amk_actual_values2);
  //  } catch (const std::runtime_error& e) {
  //   RCLCPP_ERROR(this->get_logger(), "Failed to transmit AMK actual values 2: %s", e.what());
  // }
}

void CanTxNode::can_tx_common_callback() {
  AmkTempData amk_temp_data;
  PcMainData pc_main_data;
  pc_main_data.rtd = rtd.state;


  pc_main_data.inverter_ready = inverter_on_rr & inverter_on_rl & inverter_on_fr; // inverter_on_fl
  pc_main_data.vehicle_speed = (wheel_speed_rr + wheel_speed_rl + wheel_speed_fr) /3;
  pc_main_data.torque_current = (torque_current_rr + torque_current_rl + torque_current_fr) /3;
  pc_main_data.inverter_error_fr = inverter_error_fr;
  pc_main_data.inverter_error_fl = 0;
  pc_main_data.inverter_error_rl = inverter_error_rl;
  pc_main_data.inverter_error_rr = inverter_error_rr;

  pc_main_data.inverter_on_fr = inverter_on_fr;
  pc_main_data.inverter_on_fl = 0;
  pc_main_data.inverter_on_rr = inverter_on_rr;
  pc_main_data.inverter_on_rl = inverter_on_rl;

  amk_temp_data.inverter_temp_fl = 0;
  amk_temp_data.inverter_temp_fr = inverter_temp_fr;
  amk_temp_data.inverter_temp_rl = inverter_temp_rl;
  amk_temp_data.inverter_temp_rr = inverter_temp_rr;

  amk_temp_data.motor_temp_fl = 0;
  amk_temp_data.motor_temp_fr = motor_temp_fr;
  amk_temp_data.motor_temp_rl = motor_temp_rl;
  amk_temp_data.motor_temp_rr = motor_temp_rr;


  // RCLCPP_INFO(this -> get_logger(), "motor speed %i", wheel_speed_fr );

  // RCLCPP_INFO(this -> get_logger(), "inverter fl temperaqture is %i %i %i %i", (motor_temp_rr*2),(motor_temp_rl*2), motor_temp_fl*2, motor_temp_fr*2);
  
  try {
   can_tx_common.transmit(pc_main_data);
  } catch (const std::runtime_error& e) {
   RCLCPP_ERROR(this->get_logger(), "Failed to transmit common CAN frames: %s", e.what());
 }
 amk_data_limiter_counter--;
 if(amk_data_limiter_counter == 0){
     amk_data_limiter_counter = amk_data_limiter;
    try {
      can_tx_common.transmit(amk_temp_data);
    } catch (const std::runtime_error& e) {
      RCLCPP_ERROR(this->get_logger(), "Failed to transmit AmkTempData frames: %s", e.what());
    }}
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanTxNode>());
  rclcpp::shutdown();
}
