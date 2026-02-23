#include "can_nodes/can_rx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl_interfaces;

CanRxNode::CanRxNode()
    : Node("can_rx_node")
{
    // Inicjalizacja Publisherów
    frontbox_driver_input_publisher = this->create_publisher<msg::FrontboxDriverInput>("frontbox_driver_input", 1);
    frontbox_data_publisher = this->create_publisher<msg::FrontboxData>("frontbox_data", 1);
    bms_hv_main_publisher = this->create_publisher<msg::BmsHvMain>("bms_hv_main", 1);
    bms_lv_main_publisher = this->create_publisher<msg::BmsLvMain>("bms_lv_main", 1);
    pdu_data_publisher = this->create_publisher<msg::PduData>("pdu_data", 1);
    pdu_channel_publisher = this->create_publisher<msg::PduChannel>("pdu_channel", 1);

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

    // 1. Inicjalizacja magistral CAN (SocketCAN)
    if (!can_rx_amk.Init(putm_vcl::can_interface_amk)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init AMK CAN socket");
    }
    if (!can_rx_common.Init(putm_vcl::can_interface_common)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init Common CAN socket");
    }

    // ====================================================================
    // 2. REJESTRACJA CALLBACKÓW ODBIORCZYCH (COMMON BUS)
    // ====================================================================

    can_rx_common.RegisterCallback<PUTM_CAN_M_frontbox_driver_input_t>(
        PUTM_CAN_M_FRONTBOX_DRIVER_INPUT_FRAME_ID,
        [this](const PUTM_CAN_M_frontbox_driver_input_t& can_msg) {
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
            ros_msg.front_left_suspension = can_msg.sense_suspension_fl;
            ros_msg.front_right_suspension = can_msg.sense_suspension_fr;
            ros_msg.sense_left_kill = can_msg.sense_left_kill;
            ros_msg.sense_right_kill = can_msg.sense_right_kill;
            ros_msg.sense_driver_kill = can_msg.sense_driver_kill;
            ros_msg.sense_inertia = can_msg.sense_inertia;
            ros_msg.sense_bspd = can_msg.sense_bspd;
            ros_msg.sense_overtravel = can_msg.sense_overtravel;
            ros_msg.is_braking = can_msg.is_braking;
            ros_msg.apps = can_msg.apps;
            ros_msg.apps_implausibility = can_msg.apps_implausibility;
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

    can_rx_common.RegisterCallback<PUTM_CAN_M_pdu_data_t>(
        PUTM_CAN_M_PDU_DATA_FRAME_ID,
        [this](const PUTM_CAN_M_pdu_data_t& can_msg) {
            msg::PduData ros_msg;
            ros_msg.pc_current = can_msg.pc_current;
            ros_msg.pump_current = can_msg.pump_current;
            ros_msg.fan_current = can_msg.fan_current;
            ros_msg.inverter_current = can_msg.inverter_current;
            ros_msg.fbox_current = can_msg.fbox_current;
            ros_msg.sdc_current = can_msg.sdc_current;
            ros_msg.total_current = can_msg.total_current;
            pdu_data_publisher->publish(ros_msg);
        }
    );

    can_rx_common.RegisterCallback<PUTM_CAN_M_dashboard_t>(
        PUTM_CAN_M_DASHBOARD_FRAME_ID,
        [this](const PUTM_CAN_M_dashboard_t& can_msg) {
            msg::Dashboard ros_msg;
            ros_msg.rtd_button = can_msg.rtd_button;
            ros_msg.ts_activate_button = can_msg.ts_activate_button;
            ros_msg.rfu_button = can_msg.rfu_button;
            dashboard_publisher->publish(ros_msg);
        }
    );

    // Xsens 


    // ====================================================================
    // 3. REJESTRACJA CALLBACKÓW ODBIORCZYCH (AMK BUS)
    // ====================================================================

    // Lambda pomocnicza do tworzenia wiadomości AMK z danych DBC
    auto convert_amk_actual1 = [](const auto& can_msg) {
        msg::AmkActualValues1 ros_msg;
        ros_msg.amk_status.system_ready = can_msg.system_ready;
        ros_msg.amk_status.error = can_msg.error;
        ros_msg.amk_status.warn = can_msg.warn;
        ros_msg.amk_status.quit_dc_on = can_msg.quit_dc_on;
        ros_msg.amk_status.dc_on = can_msg.dc_on;
        ros_msg.amk_status.quit_inverter_on = can_msg.quit_inverter_on;
        ros_msg.amk_status.inverter_on = can_msg.inverter_on;
        ros_msg.amk_status.derating = can_msg.derating;
        ros_msg.actual_velocity = can_msg.actual_velocity;
        ros_msg.torque_current = can_msg.torque_current;
        ros_msg.magnetizing_current = can_msg.magnetizing_current;
        return ros_msg;
    };

    // AMK Actual Values 1 (FRONT LEFT)
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values1_fl_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES1_FL_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_M_amk_actual_values1_fl_t& can_msg) {
            amk_front_left_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );

    // AMK Actual Values 1 (FRONT RIGHT)
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values1_fr_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES1_FR_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_M_amk_actual_values1_fr_t& can_msg) {
            amk_front_right_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );

    // AMK Actual Values 1 (REAR LEFT)
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values1_rl_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES1_RL_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_M_amk_actual_values1_rl_t& can_msg) {
            amk_rear_left_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );

    // AMK Actual Values 1 (REAR RIGHT)
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values1_rr_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES1_RR_FRAME_ID,
        [this, convert_amk_actual1](const PUTM_CAN_M_amk_actual_values1_rr_t& can_msg) {
            amk_rear_right_actual_values1_publisher->publish(convert_amk_actual1(can_msg));
        }
    );

    // Analogicznie dla Actual Values 2 (Temperatury)
    auto convert_amk_actual2 = [](const auto& can_msg) {
        msg::AmkActualValues2 ros_msg;
        ros_msg.temp_motor = can_msg.temp_motor;
        ros_msg.temp_inverter = can_msg.temp_inverter;
        ros_msg.error_info = can_msg.error_info;
        ros_msg.temp_igbt = can_msg.temp_igbt;
        return ros_msg;
    };

    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values2_fl_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES2_FL_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_M_amk_actual_values2_fl_t& can_msg) {
            amk_front_left_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values2_fr_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES2_FR_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_M_amk_actual_values2_fr_t& can_msg) {
            amk_front_right_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values2_rl_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES2_RL_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_M_amk_actual_values2_rl_t& can_msg) {
            amk_rear_left_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
    can_rx_amk.RegisterCallback<PUTM_CAN_M_amk_actual_values2_rr_t>(
        PUTM_CAN_M_AMK_ACTUAL_VALUES2_RR_FRAME_ID,
        [this, convert_amk_actual2](const PUTM_CAN_M_amk_actual_values2_rr_t& can_msg) {
            amk_rear_right_actual_values2_publisher->publish(convert_amk_actual2(can_msg));
        }
    );
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}