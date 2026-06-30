#pragma once

#include "PUTM_DV_CAN_LIBRARY_RAII_2024/include/can_rx.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values1.hpp"
#include "putm_vcl_interfaces/msg/amk_actual_values2.hpp"
#include "putm_vcl_interfaces/msg/dashboard.hpp"
#include "putm_vcl_interfaces/msg/frontbox_data.hpp"
#include "putm_vcl_interfaces/msg/frontbox_driver_input.hpp"
#include "putm_vcl_interfaces/msg/pdu_data.hpp"
#include "putm_vcl_interfaces/msg/pdu_channel.hpp"
#include "putm_vcl_interfaces/msg/current_sensor.hpp"
#include "putm_vcl_interfaces/msg/steering_wheel.hpp"

#include "putm_vcl_interfaces/msg/xsens_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_acceleration_hr.hpp"
#include "putm_vcl_interfaces/msg/xsens_alltitude_ellipsoid.hpp"
#include "putm_vcl_interfaces/msg/xsens_delta_q.hpp"
#include "putm_vcl_interfaces/msg/xsens_error_code.hpp"
#include "putm_vcl_interfaces/msg/xsens_euler.hpp"
#include "putm_vcl_interfaces/msg/xsens_free_acceleration.hpp"
#include "putm_vcl_interfaces/msg/xsens_inertial_data.hpp"
#include "putm_vcl_interfaces/msg/xsens_magnetic_field.hpp"
#include "putm_vcl_interfaces/msg/xsens_orientation.hpp"
#include "putm_vcl_interfaces/msg/xsens_position.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn.hpp"
#include "putm_vcl_interfaces/msg/xsens_rate_of_turn_hr.hpp"
#include "putm_vcl_interfaces/msg/xsens_status.hpp"
#include "putm_vcl_interfaces/msg/xsens_temp_and_pressure.hpp"
#include "putm_vcl_interfaces/msg/xsens_utc.hpp"
#include "putm_vcl_interfaces/msg/xsens_velocity.hpp"


#include "rclcpp/rclcpp.hpp"
#include "putm_vcl_interfaces/msg/bms_hv_main.hpp"

#include "putm_vcl_interfaces/msg/bms_lv_main.hpp"

class CanRxNode : public rclcpp::Node {
 public:
  CanRxNode();

 private:
  PUTM_CAN::CanRx can_rx_amk;
  PUTM_CAN::CanRx can_rx_common;

  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxDriverInput>::SharedPtr frontbox_driver_input_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::FrontboxData>::SharedPtr frontbox_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::BmsHvMain>::SharedPtr bms_hv_main_publisher;
  
  rclcpp::Publisher<putm_vcl_interfaces::msg::BmsLvMain>::SharedPtr bms_lv_main_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::PduData>::SharedPtr pdu_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::PduChannel>::SharedPtr pdu_channel_publisher;
    rclcpp::Publisher<putm_vcl_interfaces::msg::CurrentSensor>::SharedPtr current_sensor_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::SteeringWheel>::SharedPtr steering_wheel_publisher;


  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_left_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_front_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_front_right_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_left_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_left_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues1>::SharedPtr amk_rear_right_actual_values1_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::AmkActualValues2>::SharedPtr amk_rear_right_actual_values2_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::Dashboard>::SharedPtr dashboard_publisher;

  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensAcceleration>::SharedPtr xsens_acceleration_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensTempAndPressure>::SharedPtr xsens_temp_and_pressure_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensUtc>::SharedPtr xsens_utc_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensEuler>::SharedPtr xsens_euler_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensRateOfTurn>::SharedPtr xsens_rate_of_turn_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensOrientation>::SharedPtr xsens_orientation_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensVelocity>::SharedPtr xsens_velocity_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensInertialData>::SharedPtr xsens_inertial_data_publisher;
  rclcpp::Publisher<putm_vcl_interfaces::msg::XsensPosition>::SharedPtr xsens_position_publisher;

  rclcpp::TimerBase::SharedPtr can_rx_amk_timer;
  rclcpp::TimerBase::SharedPtr can_rx_common_timer;

  std::thread rx_thread_amk;
  std::thread rx_thread_common;

  void can_rx_amk_callback();
  void can_rx_common_callback();

  void process_common_frame(const can_frame& frame);
  void process_amk_frame(const can_frame& frame);

  template <typename T>
  putm_vcl_interfaces::msg::AmkActualValues1 create_amk_actual_values1_msg(const T& can_amk);
  template <typename T>
  putm_vcl_interfaces::msg::AmkActualValues2 create_amk_actual_values2_msg(const T& data);
};
