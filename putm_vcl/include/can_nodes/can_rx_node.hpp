#pragma once

// Nowa biblioteka CAN
#include "can_driver.hpp"
#include "PUTM_CAN_M.h"
#include "PUTM_CAN_PT.h"

// Wiadomości ROS2
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values2.hpp"
#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_data.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/pdu_data.hpp"
#include "putm_vcl_interfaces/msg/pdu_channel.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"
#include "putm_vcl_interfaces/msg/bms_lv_main.hpp"
#include "rclcpp/rclcpp.hpp"


class CanRxNode : public rclcpp::Node {
public:
  CanRxNode();

private:
  putm_ev_can::CanDriver can_rx_amk;
  putm_ev_can::CanDriver can_rx_common;

  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxData>::SharedPtr frontbox_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::BmsHvMain>::SharedPtr bms_hv_main_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::BmsLvMain>::SharedPtr bms_lv_main_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::PduData>::SharedPtr pdu_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::PduChannel>::SharedPtr pdu_channel_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_left_actual_values2_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_right_actual_values2_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_left_actual_values2_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_right_actual_values2_publisher;

};