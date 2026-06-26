#include "can_nodes/can_rx_node.hpp"
#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;

CanRxNode::CanRxNode() : Node("can_rx_node") {
    // Inicjalizacja publisherów
    frontbox_driver_input_publisher = this->create_publisher<msg::FrontboxDriverInput>("frontbox_driver_input", 1);
    frontbox_data_publisher = this->create_publisher<msg::FrontboxData>("frontbox_data", 1);
    bms_hv_main_publisher = this->create_publisher<msg::BmsHvMain>("bms_hv_main", 1);
    bms_lv_main_publisher = this->create_publisher<msg::BmsLvMain>("bms_lv_main", 1);
    pdu_data_1_publisher = this->create_publisher<msg::PduData1>("pdu_data_1", 1);
    pdu_data_2_publisher = this->create_publisher<msg::PduData2>("pdu_data_2", 1);
    pdu_channel_publisher = this->create_publisher<msg::PduChannel>("pdu_channel", 1);
    dashboard_publisher = this->create_publisher<msg::Dashboard>("dashboard", 1);
    current_sensors_data_publisher = this->create_publisher<msg::CurrentSensorData>("current_sensors_data", 1);

    amk_front_left_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/front/left/actual_values1", 1);
    amk_front_left_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/front/left/actual_values2", 1);
    amk_front_right_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/front/right/actual_values1", 1);
    amk_front_right_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/front/right/actual_values2", 1);
    amk_rear_left_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1);
    amk_rear_left_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/rear/left/actual_values2", 1);
    amk_rear_right_actual_values1_publisher = this->create_publisher<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1);
    amk_rear_right_actual_values2_publisher = this->create_publisher<msg::AmkActualValues2>("amk/rear/right/actual_values2", 1);

    // 1. Inicjalizacja magistral
    if (!can_rx_amk.Init(can_interface_amk)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init AMK CAN interface");
    }
    if (!can_rx_common.Init(can_interface_common)) {
        RCLCPP_ERROR(this->get_logger(), "Failed to init Common CAN interface");
    }

    // 2. Rejestracja zdarzeń dla COMMON (szyna główna)
    
    can_rx_common.RegisterCallback<PUTM_CAN_M_driver_input_t>(
        PUTM_CAN_M_DRIVER_INPUT_FRAME_ID,
        [this](const PUTM_CAN_M_driver_input_t& frame) {
            msg::FrontboxDriverInput ros_msg;
            ros_msg.pedal_position = frame.pedal_position;
            ros_msg.brake_pressure_front = frame.brake_pressure_front;
            ros_msg.brake_pressure_rear = frame.brake_pressure_rear;
            // TODO: Delete steering wheel position from the message
            frontbox_driver_input_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_front_data_t>(
        PUTM_CAN_M_FRONT_DATA_FRAME_ID,
        [this](const PUTM_CAN_M_front_data_t& frame) {
            msg::FrontboxData ros_msg;
            // TODO: delete front left suspension from the message
            // TODO: delete front right suspension from the message
            ros_msg.sense_left_kill = frame.sense_left_kill;
            ros_msg.sense_right_kill = frame.sense_right_kill;
            ros_msg.sense_driver_kill = frame.sense_driver_kill;
            ros_msg.sense_inertia = frame.sense_inertia;
            ros_msg.sense_bspd = frame.sense_bspd;
            ros_msg.sense_overtravel = frame.sense_overtravel;
            ros_msg.sense_suspension_fl = frame.safety_suspension_fl;
            ros_msg.sense_suspension_fr = frame.safety_suspension_fr;
            ros_msg.is_braking = frame.is_braking;
            ros_msg.apps = frame.apps;
            ros_msg.apps_implausibility = frame.apps_implausibility;
            frontbox_data_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_pdu_data_1_t>(
        PUTM_CAN_M_PDU_DATA_1_FRAME_ID,
        [this](const PUTM_CAN_M_pdu_data_1_t& frame) {
            msg::PduData1 ros_msg;
            ros_msg.pc_current = frame.pc_current;
            ros_msg.pump_current = frame.pump_current;
            ros_msg.fan_current = frame.fan_current;
            ros_msg.inverter_current = frame.inverter_current;
            pdu_data_1_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_pdu_data_2_t>(
        PUTM_CAN_M_PDU_DATA_2_FRAME_ID,
        [this](const PUTM_CAN_M_pdu_data_2_t& frame) {
            msg::PduData2 ros_msg;
            ros_msg.fbox_current = frame.fbox_current;
            ros_msg.sdc_current = frame.sdc_current;
            ros_msg.total_current = frame.total_current;
            ros_msg.reserved = frame.reserved;
            pdu_data_2_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_pdu_channnel_t>(
        PUTM_CAN_M_PDU_CHANNNEL_FRAME_ID,
        [this](const PUTM_CAN_M_pdu_channnel_t& frame) {
            msg::PduChannel ros_msg;
            ros_msg.pc_status = frame.pc_status;
            ros_msg.fan_status = frame.fan_status;
            ros_msg.pump_status = frame.pump_status;
            ros_msg.inverter_status = frame.inverter_status;
            ros_msg.fbox_status = frame.fbox_status;
            ros_msg.sdc_status = frame.sdc_status;
            ros_msg.dash_status = frame.dash_status;
            ros_msg.tsal_hv_status = frame.tsal_hv_status;
            ros_msg.rbox_diagport_brake_l_status = frame.rbox_diagport_brake_l_status;
            ros_msg.brake_ir_air_status = frame.brake_ir_air_status;
            pdu_channel_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_bms_hv_main_t>(
        PUTM_CAN_M_BMS_HV_MAIN_FRAME_ID,
        [this](const PUTM_CAN_M_bms_hv_main_t& frame) {
            msg::BmsHvMain ros_msg;
            ros_msg.voltage_sum = frame.voltage_sum;
            ros_msg.current = frame.current;
            ros_msg.temp_max = frame.temp_max;
            ros_msg.temp_avg = frame.temp_avg;
            ros_msg.soc = frame.soc;
            bms_hv_main_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_bms_lv_main_t>(
        PUTM_CAN_M_BMS_LV_MAIN_FRAME_ID,
        [this](const PUTM_CAN_M_bms_lv_main_t& frame) {
            msg::BmsLvMain ros_msg;
            ros_msg.voltage_sum = frame.voltage_sum;
            ros_msg.soc = frame.soc;
            ros_msg.temp_avg = frame.temp_avg;
            ros_msg.current = frame.current;
            bms_lv_main_publisher->publish(ros_msg);
        });

    can_rx_common.RegisterCallback<PUTM_CAN_M_dashboard_t>(
        PUTM_CAN_M_DASHBOARD_FRAME_ID,
        [this](const PUTM_CAN_M_dashboard_t& frame) {
            msg::Dashboard ros_msg;
            ros_msg.rtd_button = frame.ready_to_drive_button;
            ros_msg.ts_activate_button = frame.ts_activation_button;
            ros_msg.rfu_button = frame.user_button;
            dashboard_publisher->publish(ros_msg);
        });

    // 3. Rejestracja zdarzeń dla PT (AMK Inverters)

    // FL - AmkFrontLeftActualValues1
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_left_actual_values1_t>(
        PUTM_CAN_PT_AMK_FRONT_LEFT_ACTUAL_VALUES1_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_front_left_actual_values1_t& f) {
            msg::AmkActualValues1 msg;
            msg.amk_status.system_ready = f.amk_b_system_ready;
            msg.amk_status.error = f.amk_b_error;
            msg.amk_status.warn = f.amk_b_warn;
            msg.amk_status.dc_on = f.amk_b_dc_on;
            msg.amk_status.inverter_on = f.amk_b_inverter_on;
            msg.actual_velocity = f.amk_actual_velocity;
            msg.torque_current = f.amk_torque_current;
            amk_front_left_actual_values1_publisher->publish(msg);
        });

    // FR - AmkFrontRightActualValues1
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_right_actual_values1_t>(
        PUTM_CAN_PT_AMK_FRONT_RIGHT_ACTUAL_VALUES1_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_front_right_actual_values1_t& f) {
            msg::AmkActualValues1 msg;
            msg.amk_status.system_ready = f.amk_b_system_ready;
            msg.amk_status.error = f.amk_b_error;
            msg.amk_status.warn = f.amk_b_warn;
            msg.amk_status.dc_on = f.amk_b_dc_on;
            msg.amk_status.inverter_on = f.amk_b_inverter_on;
            msg.actual_velocity = f.amk_actual_velocity;
            msg.torque_current = f.amk_torque_current;
            amk_front_right_actual_values1_publisher->publish(msg);
        });

    // RL - AmkRearLeftActualValues1
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_left_actual_values1_t>(
        PUTM_CAN_PT_AMK_REAR_LEFT_ACTUAL_VALUES1_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_rear_left_actual_values1_t& f) {
            msg::AmkActualValues1 msg;
            msg.amk_status.system_ready = f.amk_b_system_ready;
            msg.amk_status.error = f.amk_b_error;
            msg.amk_status.warn = f.amk_b_warn;
            msg.amk_status.dc_on = f.amk_b_dc_on;
            msg.amk_status.inverter_on = f.amk_b_inverter_on;
            msg.actual_velocity = f.amk_actual_velocity;
            msg.torque_current = f.amk_torque_current;
            amk_rear_left_actual_values1_publisher->publish(msg);
        });

    // RR - AmkRearRightActualValues1
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_right_actual_values1_t>(
        PUTM_CAN_PT_AMK_REAR_RIGHT_ACTUAL_VALUES1_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_rear_right_actual_values1_t& f) {
            msg::AmkActualValues1 msg;
            msg.amk_status.system_ready = f.amk_b_system_ready;
            msg.amk_status.error = f.amk_b_error;
            msg.amk_status.warn = f.amk_b_warn;
            msg.amk_status.dc_on = f.amk_b_dc_on;
            msg.amk_status.inverter_on = f.amk_b_inverter_on;
            msg.actual_velocity = f.amk_actual_velocity;
            msg.torque_current = f.amk_torque_current;
            amk_rear_right_actual_values1_publisher->publish(msg);
        });

    // AmkActualValues2 (FL, FR, RL, RR)
    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_left_actual_values2_t>(
        PUTM_CAN_PT_AMK_FRONT_LEFT_ACTUAL_VALUES2_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_front_left_actual_values2_t& f) {
            msg::AmkActualValues2 msg;
            msg.temp_motor = f.amk_temp_motor;
            msg.temp_inverter = f.amk_temp_inverter;
            msg.error_info = f.amk_error_info;
            amk_front_left_actual_values2_publisher->publish(msg);
        });

    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_front_right_actual_values2_t>(
        PUTM_CAN_PT_AMK_FRONT_RIGHT_ACTUAL_VALUES2_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_front_right_actual_values2_t& f) {
            msg::AmkActualValues2 msg;
            msg.temp_motor = f.amk_temp_motor;
            msg.temp_inverter = f.amk_temp_inverter;
            msg.error_info = f.amk_error_info;
            amk_front_right_actual_values2_publisher->publish(msg);
        });

    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_left_actual_values2_t>(
        PUTM_CAN_PT_AMK_REAR_LEFT_ACTUAL_VALUES2_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_rear_left_actual_values2_t& f) {
            msg::AmkActualValues2 msg;
            msg.temp_motor = f.amk_temp_motor;
            msg.temp_inverter = f.amk_temp_inverter;
            msg.error_info = f.amk_error_info;
            amk_rear_left_actual_values2_publisher->publish(msg);
        });

    can_rx_amk.RegisterCallback<PUTM_CAN_PT_amk_rear_right_actual_values2_t>(
        PUTM_CAN_PT_AMK_REAR_RIGHT_ACTUAL_VALUES2_FRAME_ID,
        [this](const PUTM_CAN_PT_amk_rear_right_actual_values2_t& f) {
            msg::AmkActualValues2 msg;
            msg.temp_motor = f.amk_temp_motor;
            msg.temp_inverter = f.amk_temp_inverter;
            msg.error_info = f.amk_error_info;
            amk_rear_right_actual_values2_publisher->publish(msg);
        });
    can_rx_amk.RegisterCallback<PUTM_CAN_M_current_sensors_data_t>(
        PUTM_CAN_M_CURRENT_SENSORS_DATA_FRAME_ID,
        [this](const PUTM_CAN_M_current_sensors_data_t& frame) {
            msg::CurrentSensorData ros_msg;
            ros_msg.fl_inv_current = frame.fl_inv_current;
            ros_msg.fr_inv_current = frame.fr_inv_current;
            ros_msg.rl_inv_current = frame.rl_inv_current;
            ros_msg.rr_inv_current = frame.rr_inv_current;
            current_sensors_data_publisher->publish(ros_msg);
        });
}

int main(int argc, char** argv) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<CanRxNode>());
  rclcpp::shutdown();
}