#include "can_nodes/can_rx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl_interfaces;

CanRxNode::CanRxNode()
    : Node("can_rx_node")
{
    frontbox_driver_input_publisher = this->create_publisher<msg::FrontboxDriverInput>("frontbox_driver_input", 1);
    frontbox_data_publisher = this->create_publisher<msg::FrontboxData>("frontbox_data", 1);
    bms_hv_main_publisher = this->create_publisher<msg::BmsHvMain>("bms_hv_main", 1);
    bms_lv_main_publisher = this->create_publisher<msg::BmsLvMain>("bms_lv_main", 1);

    amk_front_left_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/front/left/actual_values1", 1);
    amk_front_left_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/front/left/actual_values2", 1);
    amk_front_right_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/front/right/actual_values1", 1);
    amk_front_right_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/front/right/actual_values2", 1);
    amk_rear_left_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1);
    amk_rear_left_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/rear/left/actual_values2", 1);
    amk_rear_right_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1);
    amk_rear_right_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/rear/right/actual_values2", 1);

    dashboard_publisher = this->create_publisher<msg::Dashboard>("dashboard", 1);
    xsens_acceleration_publisher = this->create_publisher<msg::XsensAcceleration>("xsens_acceleration", 1);
    xsens_temp_and_pressure_publisher = this->create_publisher<msg::XsensTempAndPressure>("xsens_temp", 1);
    xsens_euler_publisher = this->create_publisher<msg::XsensEuler>("xsens_euler_publisher", 1);
    xsens_rate_of_turn_publisher = this->create_publisher<msg::XsensRateOfTurn>("xsens_rate_of_turn", 1);
    xsens_orientation_publisher = this->create_publisher<msg::XsensOrientation>("xsens_orientation", 1);
    xsens_velocity_publisher = this->create_publisher<msg::XsensVelocity>("xsens_velocity", 1);
    xsens_inertial_data_publisher = this->create_publisher<msg::XsensInertialData>("xsens_dv", 1);
    xsens_position_publisher = this->create_publisher<msg::XsensPosition>("xsens_position", 1);

    if (!can_rx_amk.Init(putm_vcl::can_interface_amk)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init AMK CAN socket");
    }
    if (!can_rx_common.Init(putm_vcl::can_interface_common)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init Common CAN socket");
    }

    can_rx_common.RegisterCallback<PUTM_CAN_M_driver_input_t>(
        PUTM_CAN_M_DRIVER_INPUT_FRAME_ID,
        [this](const PUTM_CAN_M_driver_input_t& can_msg) {
            msg::FrontboxDriverInput ros_msg;
            ros_msg.pedal_position = can_msg.pedal_position;
            ros_msg.brake_pressure_front = can_msg.brake_pressure_front;
            ros_msg.brake_pressure_rear = can_msg.brake_pressure_rear;
            ros_msg.steering_wheel_position = can_msg.steering_wheel_position;
            frontbox_driver_input_publisher->publish(ros_msg);
        }
    );

    can_rx_common.RegisterCallback<PUTM_CAN_M_front_data_t>(
        PUTM_CAN_M_FRONT_DATA_FRAME_ID,
        [this](const PUTM_CAN_M_front_data_t& can_msg) {
            msg::FrontboxData ros_msg;
            ros_msg.front_left_suspension = can_msg.safety_suspension_fl;
            ros_msg.front_right_suspension = can_msg.safety_suspension_fr;
            ros_msg.sense_left_kill = can_msg.sense_left_kill;
            ros_msg.sense_right_kill = can_msg.sense_right_kill;
            ros_msg.sense_driver_kill = can_msg.sense_driver_kill;
            ros_msg.sense_inertia = can_msg.sense_inertia;
            ros_msg.sense_bspd = can_msg.sense_bspd;
            ros_msg.sense_overtravel = can_msg.sense_overtravel;
            frontbox_data_publisher->publish(ros_msg);
        }
    );

    can_rx_common.RegisterCallback<PUTM_CAN_M_bms_hv_main_t>(
        PUTM_CAN_M_BMS_HV_MAIN_FRAME_ID,
        [this](const PUTM_CAN_M_bms_hv_main_t& can_msg) {
            msg::BmsHvMain ros_msg;
            ros_msg.voltage_sum = can_msg.voltage_sum;
            ros_msg.current = can_msg.current;
            ros_msg.temp_max = can_msg.temp_max;
            ros_msg.temp_avg = can_msg.temp_avg;
            ros_msg.soc = can_msg.soc;
            bms_hv_main_publisher->publish(ros_msg);
        }
    );

    can_rx_common.RegisterCallback<PUTM_CAN_M_bms_lv_main_t>(
        PUTM_CAN_M_BMS_LV_MAIN_FRAME_ID,
        [this](const PUTM_CAN_M_bms_lv_main_t& can_msg) {
            msg::BmsLvMain ros_msg;
            ros_msg.voltage_sum = can_msg.voltage_sum;
            ros_msg.current = can_msg.current;
            ros_msg.temp_avg = can_msg.temp_avg;
            ros_msg.soc = can_msg.soc;
            bms_lv_main_publisher->publish(ros_msg);
        }
    );

    can_rx_common.RegisterCallback<PUTM_CAN_M_dashboard_t>(
        PUTM_CAN_M_DASHBOARD_FRAME_ID,
        [this](const PUTM_CAN_M_dashboard_t& can_msg) {
            msg::Dashboard ros_msg;
            ros_msg.rtd_button = can_msg.ready_to_drive_button;
            ros_msg.ts_activate_button = can_msg.ts_activation_button;
            ros_msg.rfu_button = can_msg.user_button;
            dashboard_publisher->publish(ros_msg);
        }
    );

    auto convert_amk_actual1 = [](const auto& can_msg) {
        msg::AmkActualValues1 ros_msg;
        ros_msg.amk_status.system_ready = can_msg.amk_b_system_ready;
        ros_msg.amk_status.error = can_msg.amk_b_error;
        ros_msg.amk_status.warn = can_msg.amk_b_warn;
        ros_msg.amk_status.dc_on = can_msg.amk_b_dc_on;
        ros_msg.amk_status.inverter_on = can_msg.amk_b_inverter_on;
        ros_msg.actual_velocity = can_msg.amk_actual_velocity;
        ros_msg.torque_current = can_msg.amk_torque_current;
        return ros_msg;
    };

    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_left_actual_values1_t>(
        PUTM_CAN_PT_AMK_FRONT_LEFT_ACTUAL_VALUES1_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_PT_amk_front_left_actual_values1_t& can_msg) {
            amk_front_left_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_right_actual_values1_t>(
        PUTM_CAN_PT_AMK_FRONT_RIGHT_ACTUAL_VALUES1_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_PT_amk_front_right_actual_values1_t& can_msg) {
            amk_front_right_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_left_actual_values1_t>(
        PUTM_CAN_PT_AMK_REAR_LEFT_ACTUAL_VALUES1_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_PT_amk_rear_left_actual_values1_t& can_msg) {
            amk_rear_left_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_right_actual_values1_t>(
        PUTM_CAN_PT_AMK_REAR_RIGHT_ACTUAL_VALUES1_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_PT_amk_rear_right_actual_values1_t& can_msg) {
            amk_rear_right_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );

    auto convert_amk_actual2 = [](const auto& can_msg) {
        msg::AmkActualValues2 ros_msg;
        ros_msg.temp_motor = can_msg.amk_temp_motor;
        ros_msg.temp_inverter = can_msg.amk_temp_inverter;
        ros_msg.error_info = can_msg.amk_diagnosis_no;
        return ros_msg;
    };

    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_left_actual_values2_t>(
        PUTM_CAN_PT_AMK_FRONT_LEFT_ACTUAL_VALUES2_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_PT_amk_front_left_actual_values2_t& can_msg) {
            amk_front_left_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_right_actual_values2_t>(
        PUTM_CAN_PT_AMK_FRONT_RIGHT_ACTUAL_VALUES2_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_PT_amk_front_right_actual_values2_t& can_msg) {
            amk_front_right_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_left_actual_values2_t>(
        PUTM_CAN_PT_AMK_REAR_LEFT_ACTUAL_VALUES2_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_PT_amk_rear_left_actual_values2_t& can_msg) {
            amk_rear_left_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_right_actual_values2_t>(
        PUTM_CAN_PT_AMK_REAR_RIGHT_ACTUAL_VALUES2_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_PT_amk_rear_right_actual_values2_t& can_msg) {
            amk_rear_right_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}