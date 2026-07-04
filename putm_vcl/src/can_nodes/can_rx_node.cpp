#include "can_nodes/can_rx_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace PUTM_CAN;
using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;

CanRxNode::CanRxNode()
    : Node("can_rx_node"),
      can_rx_amk(can_interface_amk, NO_TIMEOUT),
      can_rx_common(can_interface_common, NO_TIMEOUT),

      frontbox_driver_input_publisher(this->create_publisher<msg::FrontboxDriverInput>("frontbox_driver_input", 1)),
      frontbox_data_publisher(this->create_publisher<msg::FrontboxData>("frontbox_data", 1)),
      bms_hv_main_publisher(this->create_publisher<msg::BmsHvMain>("bms_hv_main", 1)),
      bms_lv_main_publisher(this->create_publisher<msg::BmsLvMain>("bms_lv_main", 1)),
      
      pdu_data_publisher(this->create_publisher<msg::PduData>("pdu_data",1)),
      pdu_channel_publisher(this->create_publisher<msg::PduChannel>("pdu_channel",1)),
      current_sensor_publisher(this->create_publisher<msg::CurrentSensor>("current_sensor",1)),
      ivt_current_publisher(this->create_publisher<msg::IvtCurrent>("ivt_current", 1)),
      steering_wheel_publisher(this->create_publisher<msg::SteeringWheel>("steering_wheel",1)),

      amk_front_left_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("amk/front/left/actual_values1", 1)),
      amk_front_left_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("amk/front/left/actual_values2", 1)),

      amk_front_right_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("amk/front/right/actual_values1", 1)),
      amk_front_right_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("amk/front/right/actual_values2", 1)),

      amk_rear_left_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1)),
      amk_rear_left_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("amk/rear/left/actual_values2", 1)),

      amk_rear_right_actual_values1_publisher(this->create_publisher<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1)),
      amk_rear_right_actual_values2_publisher(this->create_publisher<msg::AmkActualValues2>("amk/rear/right/actual_values2", 1)),

      dashboard_publisher(this->create_publisher<msg::Dashboard>("dashboard", 1)),

      xsens_acceleration_publisher(this->create_publisher<msg::XsensAcceleration>("xsens_acceleration", 1)),
      xsens_temp_and_pressure_publisher(this->create_publisher<msg::XsensTempAndPressure>("xsens_temp", 1)),
      xsens_utc_publisher(this->create_publisher<msg::XsensUtc>("xsens_utc", 1)),
      xsens_euler_publisher(this->create_publisher<msg::XsensEuler>("xsens_euler_publisher", 1)),
      xsens_rate_of_turn_publisher(this->create_publisher<msg::XsensRateOfTurn>("xsens_rate_of_turn", 1)),
      xsens_orientation_publisher(this->create_publisher<msg::XsensOrientation>("xsens_orientation", 1)),
      xsens_velocity_publisher(this->create_publisher<msg::XsensVelocity>("xsens_velocity", 1)),
      xsens_inertial_data_publisher(this->create_publisher<msg::XsensInertialData>("xsens_dv", 1)),
      xsens_position_publisher(this->create_publisher<msg::XsensPosition>("xsens_position", 1))
{
  rx_thread_common = std::thread([this]() {
    while (rclcpp::ok()) {
      can_frame frame;
      try {
        frame = can_rx_common.receive();
      } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "Common CAN error: %s", e.what());
        continue;
      }
      process_common_frame(frame);
    }
  });

  rx_thread_amk = std::thread([this]() {
    while (rclcpp::ok()) {
      can_frame frame;
      try {
        frame = can_rx_amk.receive();
      } catch (const std::runtime_error& e) {
        RCLCPP_ERROR(this->get_logger(), "AMK CAN error: %s", e.what());
        continue;
      }
      process_amk_frame(frame);
    }
  });
}

// CanRxNode::~CanRxNode() {
//   if (rx_thread_common.joinable()) rx_thread_common.join();
//   if (rx_thread_amk.joinable()) rx_thread_amk.join();
// }

void CanRxNode::process_common_frame(const can_frame& frame) {
  try {
    switch (frame.can_id) {
      case can_id<FrontboxDriverInput>: {
        auto can_frontbox_driver_input = convert<FrontboxDriverInput>(frame);
        msg::FrontboxDriverInput frontbox_driver_input;
        frontbox_driver_input.pedal_position = can_frontbox_driver_input.pedal_position;
        frontbox_driver_input.brake_pressure_front = can_frontbox_driver_input.brake_pressure_front;
        frontbox_driver_input.brake_pressure_rear = can_frontbox_driver_input.brake_pressure_rear;
        frontbox_driver_input.steering_wheel_position = can_frontbox_driver_input.steering_wheel_position;
        frontbox_driver_input_publisher->publish(frontbox_driver_input);
        break;
      }

      case can_id<FrontboxData>: {
        auto can_frontbox_data = convert<FrontboxData>(frame);
        msg::FrontboxData frontbox_data;
        frontbox_data.front_left_suspension = can_frontbox_data.front_left_suspension;
        frontbox_data.front_right_suspension = can_frontbox_data.front_right_suspension;
        frontbox_data.sense_left_kill = can_frontbox_data.sense_left_kill;
        frontbox_data.sense_right_kill = can_frontbox_data.sense_right_kill;
        frontbox_data.sense_driver_kill = can_frontbox_data.sense_driver_kill;
        frontbox_data.sense_inertia = can_frontbox_data.sense_inertia;
        frontbox_data.sense_bspd = can_frontbox_data.sense_bspd;
        frontbox_data.sense_overtravel = can_frontbox_data.sense_overtravel;
        frontbox_data.sense_suspension_fl = can_frontbox_data.sense_suspension_fl;
        frontbox_data.sense_suspension_fr = can_frontbox_data.sense_suspension_fr;
        frontbox_data.is_braking = can_frontbox_data.is_braking;
        frontbox_data.apps = can_frontbox_data.apps;
        frontbox_data.apps_implausibility = can_frontbox_data.apps_implausibility;
        frontbox_data_publisher->publish(frontbox_data);
        break;
      }

      case can_id<PduData>: {
        auto can_pdu_data = convert<PduData>(frame);
        msg::PduData pdu_data;
        pdu_data.pc_current = can_pdu_data.pc_current;
        pdu_data.pump_current = can_pdu_data.pump_current;
        pdu_data.fan_current = can_pdu_data.fan_current;
        pdu_data.inverter_current = can_pdu_data.inverter_current;
        pdu_data.fbox_current = can_pdu_data.fbox_current;
        pdu_data.sdc_current = can_pdu_data.sdc_current;
        pdu_data.total_current = can_pdu_data.total_current;
        pdu_data_publisher->publish(pdu_data);
        break;
      }
      case can_id<CurrentSensor>: {
        auto can_current_sensor = convert<CurrentSensor>(frame);
        msg::CurrentSensor current_sensor;
        current_sensor.fl_inv_current = can_current_sensor.fl_inv_current;
        current_sensor.fr_inv_current = can_current_sensor.fr_inv_current;
        current_sensor.rl_inv_current = can_current_sensor.rl_inv_current;
        current_sensor.rr_inv_current = can_current_sensor.rr_inv_current;
        current_sensor_publisher->publish(current_sensor);
        break;
      }
      case can_id<IVT_Msg_Result_I>: {
        auto can_ivt = convert<IVT_Msg_Result_I>(frame);
        msg::IvtCurrent ivt_current;
        ivt_current.current = decode_ivt_result_i(can_ivt);
        ivt_current_publisher->publish(ivt_current);
        break;
      }
      case can_id<SteeringWheel>: {
        auto can_steering_wheel = convert<SteeringWheel>(frame);
        msg::SteeringWheel steering_wheel;
        steering_wheel.steering_wheel_position = can_steering_wheel.steering_wheel_position;
        steering_wheel.measurement2 = can_steering_wheel.measurement2;
        steering_wheel.measurement3 = can_steering_wheel.measurement3;
        steering_wheel.measurement4 = can_steering_wheel.measurement4;
        steering_wheel_publisher->publish(steering_wheel);
        break;
      }
      case can_id<PduChannel>: {
        auto can_pdu_channel = convert<PduChannel>(frame);
        msg::PduChannel pdu_channel;
        pdu_channel.pc_status = can_pdu_channel.pc_status;
        pdu_channel.fan_status = can_pdu_channel.fan_status;
        pdu_channel.pump_status = can_pdu_channel.pump_status;
        pdu_channel.inverter_status = can_pdu_channel.inverter_status;
        pdu_channel.fbox_status = can_pdu_channel.fbox_status;
        pdu_channel.sdc_status = can_pdu_channel.sdc_status;
        pdu_channel.dash_status = can_pdu_channel.dash_status;
        pdu_channel.tsal_hv_status = can_pdu_channel.tsal_hv_status;
        pdu_channel.rbox_diagport_brake_l_status = can_pdu_channel.rbox_diagport_brake_l_status;
        pdu_channel.brake_ir_air_status = can_pdu_channel.brake_ir_air_status;
        pdu_channel_publisher->publish(pdu_channel);
        break;
      }

      case can_id<BmsHvMain>: {
        auto can_bms_hv_main = convert<BmsHvMain>(frame);
        msg::BmsHvMain bms_hv_main;
        bms_hv_main.voltage_sum = can_bms_hv_main.voltage_sum;
        bms_hv_main.current = can_bms_hv_main.current;
        bms_hv_main.temp_max = can_bms_hv_main.temp_max;
        bms_hv_main.temp_avg = can_bms_hv_main.temp_avg;
        bms_hv_main.soc = can_bms_hv_main.soc;
        bms_hv_main_publisher->publish(bms_hv_main);
        break;
      }

      case can_id<BmsLvMain>: {
        auto can_bms_lv_main = convert<BmsLvMain>(frame);
        msg::BmsLvMain bms_lv_main;
        bms_lv_main.voltage_sum = can_bms_lv_main.voltage_sum;
        bms_lv_main.soc = can_bms_lv_main.soc;
        bms_lv_main.temp_avg = can_bms_lv_main.temp_avg;
        bms_lv_main.current = can_bms_lv_main.current;
        bms_lv_main_publisher->publish(bms_lv_main);
        break;
      }

      case can_id<Dashboard>: {
        auto can_dashboard = convert<Dashboard>(frame);
        msg::Dashboard dashboard;
        dashboard.rtd_button = can_dashboard.rtd_button;
        dashboard.ts_activate_button = can_dashboard.ts_activate_button;
        dashboard.rfu_button = can_dashboard.rfu_button;
        dashboard_publisher->publish(dashboard);
        break;
      }

      case can_id<XsensAcceleration>: {
        msg::XsensAcceleration xsens_acceleration;
        double scale = 1.0 / (1 << 8);
        float *acc_arr[] = {&xsens_acceleration.acc_x, &xsens_acceleration.acc_y, &xsens_acceleration.acc_z};
        for (size_t i = 0; i < 3; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *acc_arr[i] = static_cast<float>(value * scale);
        }
        xsens_acceleration_publisher->publish(xsens_acceleration);
        break;
      }

      case can_id<XsensAccelerationHighRate>: { break; }
      case can_id<XsensAltitudeEllipsoid>:    { break; }
      case can_id<XsensDeltaQ>:               { break; }
      case can_id<XsensError>:                { break; }

      case can_id<XsensEuler>: {
        msg::XsensEuler euler;
        double scale = 1.0 / (1 << 7);
        float *euler_arr[] = {&euler.roll, &euler.pitch, &euler.yaw};
        for (size_t i = 0; i < 3; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *euler_arr[i] = static_cast<float>(value * scale);
        }
        xsens_euler_publisher->publish(euler);
        break;
      }

      case can_id<XsensFreeAcceleration>: { break; }

      case can_id<XsensInertialData>: {
        msg::XsensInertialData dv;
        uint8_t exponent = frame.data[6];
        double scale = 1.0 / (1 << exponent);
        float *dv_arr[] = {&dv.x, &dv.y, &dv.z};
        for (size_t i = 0; i < 3; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *dv_arr[i] = static_cast<float>(value * scale);
        }
        xsens_inertial_data_publisher->publish(dv);
        break;
      }

      case can_id<XsensMagneticField>: { break; }

      case can_id<XsensOrientation>: {
        msg::XsensOrientation q;
        double scale = 1.0 / ((1 << 15) - 1);
        float *q_arr[] = {&q.q0, &q.q1, &q.q2, &q.q3};
        for (size_t i = 0; i < 4; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *q_arr[i] = static_cast<float>(value * scale);
        }
        xsens_orientation_publisher->publish(q);
        break;
      }

      case can_id<XsensPosition>: {
        msg::XsensPosition latlon;
        uint32_t latitude = 0;
        uint32_t longitude = 0;
        double scale_lat = 1.0 / (1 << 24);
        double scale_lon = 1.0 / (1 << 23);
        latitude  |= static_cast<uint32_t>(frame.data[0]) << 24;
        latitude  |= static_cast<uint32_t>(frame.data[1]) << 16;
        latitude  |= static_cast<uint32_t>(frame.data[2]) << 8;
        latitude  |= static_cast<uint32_t>(frame.data[3]);
        longitude |= static_cast<uint32_t>(frame.data[4]) << 24;
        longitude |= static_cast<uint32_t>(frame.data[5]) << 16;
        longitude |= static_cast<uint32_t>(frame.data[6]) << 8;
        longitude |= static_cast<uint32_t>(frame.data[7]);
        latlon.latitude  = static_cast<double>(latitude  * scale_lat);
        latlon.longitude = static_cast<double>(longitude * scale_lon);
        xsens_position_publisher->publish(latlon);
        break;
      }

      case can_id<XsensRateOfTurn>: {
        msg::XsensRateOfTurn gyro;
        double scale = 1.0 / (1 << 9);
        float *gyro_arr[] = {&gyro.gyr_x, &gyro.gyr_y, &gyro.gyr_z};
        for (size_t i = 0; i < 3; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *gyro_arr[i] = static_cast<float>(value * scale);
        }
        xsens_rate_of_turn_publisher->publish(gyro);
        break;
      }

      case can_id<XsensRateOfTurnHighRate>: { break; }
      case can_id<XsensStatus>:             { break; }

      case can_id<XsensTemperatureAndPressure>: {
        msg::XsensTempAndPressure xsens_t_and_p;
        uint16_t temperature = 0;
        double scale = 1.0 / (1 << 8);
        temperature |= static_cast<uint16_t>(frame.data[0]) << 8;
        temperature |= static_cast<uint16_t>(frame.data[1]);
        xsens_t_and_p.temperature = static_cast<float>(temperature * scale);
        xsens_temp_and_pressure_publisher->publish(xsens_t_and_p);
        break;
      }

      case can_id<XsensUtc>: {
        // not implemented
        break;
      }

      case can_id<XsensVelocity>: {
        msg::XsensVelocity vel;
        double scale = 1.0 / (1 << 6);
        float *vel_arr[3] = {&vel.x, &vel.y, &vel.z};
        for (size_t i = 0; i < 3; ++i) {
          int16_t value = static_cast<int16_t>((frame.data[2 * i] << 8) | frame.data[2 * i + 1]);
          *vel_arr[i] = static_cast<float>(value * scale);
        }
        xsens_velocity_publisher->publish(vel);
        break;
      }
    }
  } catch (const std::runtime_error& e) {
    // RCLCPP_ERROR(this->get_logger(), "Failed to convert common CAN frame: %s", e.what());
  }
}

void CanRxNode::process_amk_frame(const can_frame& frame) {
  try {
    switch (frame.can_id) {
      case can_id<AmkFrontLeftActualValues1>: {
        auto can_amk = convert<AmkFrontLeftActualValues1>(frame);
        amk_front_left_actual_values1_publisher->publish(create_amk_actual_values1_msg(can_amk));
        break;
      }
      case can_id<AmkFrontRightActualValues1>: {
        auto can_amk = convert<AmkFrontRightActualValues1>(frame);
        amk_front_right_actual_values1_publisher->publish(create_amk_actual_values1_msg(can_amk));
        break;
      }
      case can_id<AmkRearLeftActualValues1>: {
        auto can_amk = convert<AmkRearLeftActualValues1>(frame);
        amk_rear_left_actual_values1_publisher->publish(create_amk_actual_values1_msg(can_amk));
        break;
      }
      case can_id<AmkRearRightActualValues1>: {
        auto can_amk = convert<AmkRearRightActualValues1>(frame);
        amk_rear_right_actual_values1_publisher->publish(create_amk_actual_values1_msg(can_amk));
        break;
      }
      case can_id<AmkFrontLeftActualValues2>: {
        auto can_amk = convert<AmkFrontLeftActualValues2>(frame);
        amk_front_left_actual_values2_publisher->publish(create_amk_actual_values2_msg(can_amk));
        break;
      }
      case can_id<AmkFrontRightActualValues2>: {
        auto can_amk = convert<AmkFrontRightActualValues2>(frame);
        amk_front_right_actual_values2_publisher->publish(create_amk_actual_values2_msg(can_amk));
        break;
      }
      case can_id<AmkRearLeftActualValues2>: {
        auto can_amk = convert<AmkRearLeftActualValues2>(frame);
        amk_rear_left_actual_values2_publisher->publish(create_amk_actual_values2_msg(can_amk));
        break;
      }
      case can_id<AmkRearRightActualValues2>: {
        auto can_amk = convert<AmkRearRightActualValues2>(frame);
        amk_rear_right_actual_values2_publisher->publish(create_amk_actual_values2_msg(can_amk));
        break;
      }
    }
  } catch (const std::runtime_error& e) {
    RCLCPP_ERROR(this->get_logger(), "Failed to convert AMK CAN frame: %s", e.what());
  }
}

template <typename T>
msg::AmkActualValues1 CanRxNode::create_amk_actual_values1_msg(const T& can_amk) {
  msg::AmkActualValues1 amk_actual_values1;
  amk_actual_values1.amk_status.system_ready    = can_amk.amk_status.system_ready;
  amk_actual_values1.amk_status.error           = can_amk.amk_status.error;
  amk_actual_values1.amk_status.warn            = can_amk.amk_status.warn;
  amk_actual_values1.amk_status.quit_dc_on      = can_amk.amk_status.quit_dc_on;
  amk_actual_values1.amk_status.dc_on           = can_amk.amk_status.dc_on;
  amk_actual_values1.amk_status.quit_inverter_on = can_amk.amk_status.quit_inverter_on;
  amk_actual_values1.amk_status.inverter_on     = can_amk.amk_status.inverter_on;
  amk_actual_values1.amk_status.derating        = can_amk.amk_status.derating;
  amk_actual_values1.actual_velocity            = can_amk.actual_velocity;
  amk_actual_values1.torque_current             = can_amk.torque_current;
  amk_actual_values1.magnetizing_current        = can_amk.magnetizing_current;
  return amk_actual_values1;
}

template <typename T>
msg::AmkActualValues2 CanRxNode::create_amk_actual_values2_msg(const T& can_amk) {
  msg::AmkActualValues2 amk_actual_values2;
  amk_actual_values2.temp_motor    = can_amk.temp_motor;
  amk_actual_values2.temp_inverter = can_amk.temp_inverter;
  amk_actual_values2.error_info    = can_amk.error_info;
  amk_actual_values2.temp_igbt     = can_amk.temp_igbt;
  return amk_actual_values2;
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}