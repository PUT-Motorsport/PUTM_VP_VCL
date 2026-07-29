#pragma once

#include "can_driver.hpp"
#include "PUTM_CAN_M.h"
#include "PUTM_CAN_PT.h"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values2.hpp"
#include "putm_vcl_interfaces/msg/amk_setpoints.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include "putm_vcl_interfaces/msg/lap_timer.hpp"

#include "rclcpp/rclcpp.hpp"

class CanTxNode : public rclcpp::Node {
public:
  CanTxNode();

private:
  putm_ev_can::CanDriver can_tx_amk;
  putm_ev_can::CanDriver can_tx_common;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_front_right_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_left_setpoints_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkSetpoints>::SharedPtr amk_rear_right_setpoints_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_left_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_right_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_left_actual_values2_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_right_actual_values2_subscriber;

  rclcpp::Subscription<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::LapTimer>::SharedPtr lap_timer_subscriber;
  rclcpp::TimerBase::SharedPtr can_tx_common_timer;

  int16_t torque_current_rr = 0, torque_current_rl = 0, torque_current_fr = 0, torque_current_fl = 0;
  int16_t wheel_speed_rr = 0, wheel_speed_rl = 0, wheel_speed_fr = 0, wheel_speed_fl = 0;
  
  bool inverter_ready_rr = 0, inverter_ready_rl = 0, inverter_ready_fr = 0, inverter_ready_fl = 0;
  bool inverter_on_rr = 0, inverter_on_rl = 0, inverter_on_fr = 0, inverter_on_fl = 0;
  bool inverter_error_rr = 0, inverter_error_rl = 0, inverter_error_fr = 0, inverter_error_fl = 0;

  int8_t inverter_temp_rr = 0, inverter_temp_rl = 0, inverter_temp_fr = 0, inverter_temp_fl = 0;
  int8_t motor_temp_rr = 0, motor_temp_rl = 0, motor_temp_fr = 0, motor_temp_fl = 0;

  const uint8_t amk_data_limiter = 60;
  uint8_t amk_data_limiter_counter = 0;

  putm_vcl_interfaces::msg::Rtd rtd;

  void rtd_callback(const putm_vcl_interfaces::msg::Rtd msg);
  void lap_timer_callback(const putm_vcl_interfaces::msg::LapTimer msg);
  void can_tx_common_callback();

  void amk_fl_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_fr_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_rl_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);
  void amk_rr_setpoints_callback(const putm_vcl_interfaces::msg::AmkSetpoints msg);

  void amk_fl_actual1_callback(const putm_vcl_interfaces::msg::AmkActualValues1 msg);
  void amk_fr_actual1_callback(const putm_vcl_interfaces::msg::AmkActualValues1 msg);
  void amk_rl_actual1_callback(const putm_vcl_interfaces::msg::AmkActualValues1 msg);
  void amk_rr_actual1_callback(const putm_vcl_interfaces::msg::AmkActualValues1 msg);

  void amk_fl_actual2_callback(const putm_vcl_interfaces::msg::AmkActualValues2 msg);
  void amk_fr_actual2_callback(const putm_vcl_interfaces::msg::AmkActualValues2 msg);
  void amk_rl_actual2_callback(const putm_vcl_interfaces::msg::AmkActualValues2 msg);
  void amk_rr_actual2_callback(const putm_vcl_interfaces::msg::AmkActualValues2 msg);
};