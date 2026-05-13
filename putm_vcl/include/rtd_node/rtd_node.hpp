#pragma once

#include "putm_vcl_interfaces/msg/state_machine.hpp"
#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/rtd.hpp"
#include <putm_vcl_interfaces/msg/amk_actual_values1.hpp>
#include "rclcpp/rclcpp.hpp"

class RtdNode : public rclcpp::Node {
 public:
  RtdNode();

 private:

  rclcpp::QoS qos_;

  putm_vcl_interfaces::msg::Rtd rtd;
  putm_vcl_interfaces::msg::StateMachine state_machine;
  putm_vcl_interfaces::msg::FrontboxDriverInput frontbox_driver_input;
  putm_vcl_interfaces::msg::Dashboard dashboard;

  putm_vcl_interfaces::msg::AmkActualValues1   amk_front_left_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1   amk_front_right_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1   amk_rear_left_actual_values1;
  putm_vcl_interfaces::msg::AmkActualValues1   amk_rear_right_actual_values1;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Rtd>::SharedPtr rtd_publisher;
  rclcpp::Subscription<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_subscription;
  rclcpp::Subscription<putm_vcl_interfaces::msg::StateMachine>::SharedPtr state_machine_subscriber;
  rclcpp::TimerBase::SharedPtr rtd_timer;

  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_subscriber;
  rclcpp::Subscription<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_subscriber;

  std::function<void(const putm_vcl_interfaces::msg::AmkActualValues1::SharedPtr)> amk_actual_values1_callback_factory(
      putm_vcl_interfaces::msg::AmkActualValues1& target);

  void rtd_callback();
  void frontbox_driver_input_callback(const putm_vcl_interfaces::msg::FrontboxDriverInput::SharedPtr msg);
  void dashboard_callback(const putm_vcl_interfaces::msg::Dashboard::SharedPtr msg);
  void state_machine_callback(const putm_vcl_interfaces::msg::StateMachine::SharedPtr msg);
};
