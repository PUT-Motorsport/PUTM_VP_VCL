#include "amk_node/amk_node.hpp"

#include "putm_vcl/putm_vcl.hpp"

using namespace putm_vcl;
using namespace putm_vcl_interfaces;
using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

AmkNode::AmkNode()
    : Node("amk_node"),
      state(StateMachine::UNDEFINED),
      state_machine_publisher(this->create_publisher<msg::StateMachine>("state_machine", 1)),
      amk_front_left_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/front/left/setpoints", 1)),
      amk_front_right_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/front/right/setpoints", 1)),
      amk_rear_left_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/rear/left/setpoints", 1)),
      amk_rear_right_setpoints_publisher(this->create_publisher<msg::AmkSetpoints>("amk/rear/right/setpoints", 1)),
      // clang-format off
      amk_front_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_left_actual_values1))),
      amk_front_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/front/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_front_right_actual_values1))),
      amk_rear_left_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/left/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_left_actual_values1))),
      amk_rear_right_actual_values1_subscriber(
          this->create_subscription<msg::AmkActualValues1>("amk/rear/right/actual_values1", 1, 
                                                           amk_actual_values1_callback_factory(amk_rear_right_actual_values1))),
      // clang-format on
      rtd_subscriber(this->create_subscription<msg::Rtd>("rtd", 1, std::bind(&AmkNode::rtd_callback, this, _1))),
      setpoints_subscriber(this->create_subscription<msg::Setpoints>("setpoints", 1, std::bind(&AmkNode::setpoints_callback, this, _1))),

      setpoints_watchdog(this->create_wall_timer(500ms, std::bind(&AmkNode::setpoints_watchdog_callback, this))),
      amk_state_machine_watchdog(this->create_wall_timer(5000ms, std::bind(&AmkNode::amk_state_machine_watchdog_callback, this))),
      amk_state_machine_timer(this->create_wall_timer(5ms, std::bind(&AmkNode::amk_state_machine_callback, this))),
      amk_setpoints_timer(this->create_wall_timer(2ms, std::bind(&AmkNode::amk_setpoints_callback, this)))
{
  /*Transition functions*/
  add_transition(StateMachine::UNDEFINED, StateMachine::ERROR_RESET, std::bind(&AmkNode::check_inv_errors, this));
  add_transition(StateMachine::UNDEFINED, StateMachine::IDLING, std::bind(&AmkNode::system_ready, this));
  add_transition(StateMachine::IDLING, StateMachine::ERROR_RESET, std::bind(&AmkNode::check_inv_errors, this));
  add_transition(StateMachine::IDLING, StateMachine::DC_STARTUP, std::bind(&AmkNode::check_rtd, this));
  add_transition(StateMachine::DC_STARTUP, StateMachine::INVERTER_STARTUP, std::bind(&AmkNode::check_dc_on, this));
  add_transition(StateMachine::INVERTER_STARTUP, StateMachine::TORQUE_CONTROL, std::bind(&AmkNode::check_quit_inverter_on, this));
  add_transition(StateMachine::TORQUE_CONTROL, StateMachine::SWITCH_OFF, [this]()
                 { return !check_rtd(); });
  add_transition(StateMachine::TORQUE_CONTROL, StateMachine::SWITCH_OFF, [this]()
                 { return !check_inv_on(); });
  add_transition(StateMachine::SWITCH_OFF, StateMachine::IDLING, std::bind(&AmkNode::all_inv_on, this));
  add_transition(StateMachine::ERROR_RESET, StateMachine::IDLING, std::bind(&AmkNode::system_ready, this));

  setpoints_watchdog->cancel();
  amk_state_machine_watchdog->cancel();
}

void AmkNode::rtd_callback(const msg::Rtd::SharedPtr msg) { rtd = *msg; }

void AmkNode::setpoints_callback(const msg::Setpoints::SharedPtr msg)
{
  setpoints_watchdog->cancel();
  setpoints = *msg;
  setpoints_watchdog->reset();
}
std::function<void(const msg::AmkActualValues1::SharedPtr msg)> AmkNode::amk_actual_values1_callback_factory(msg::AmkActualValues1 &target)
{
  return [this, &target](const putm_vcl_interfaces::msg::AmkActualValues1::SharedPtr msg)
  { target = *msg; };
}

void AmkNode::setpoints_watchdog_callback()
{
  RCLCPP_WARN(this->get_logger(), "Setpoints watchdog triggered");
  setpoints.front_left.torque = 0;
  setpoints.front_right.torque = 0;
  setpoints.rear_left.torque = 0;
  setpoints.rear_right.torque = 0;
  setpoints_watchdog->cancel();
}
void AmkNode::amk_state_machine_watchdog_callback()
{
  RCLCPP_WARN(this->get_logger(), "State machine watchdog triggered");
  state = StateMachine::SWITCH_OFF;
  amk_state_machine_watchdog->cancel();
  state_machine.state = 1;
  state_machine_publisher->publish(state_machine);
}

void AmkNode::amk_setpoints_callback()
{
  amk_front_left_setpoints_publisher->publish(amk_front_left_setpoints);
  amk_front_right_setpoints_publisher->publish(amk_front_right_setpoints);
  amk_rear_left_setpoints_publisher->publish(amk_rear_left_setpoints);
  amk_rear_right_setpoints_publisher->publish(amk_rear_right_setpoints);
}

bool AmkNode::check_rtd()
{
  return rtd.state == true;
}

bool AmkNode::check_dc_on()
{
  return (amk_front_left_actual_values1.amk_status.quit_dc_on && amk_front_right_actual_values1.amk_status.quit_dc_on && amk_rear_left_actual_values1.amk_status.quit_dc_on && amk_rear_right_actual_values1.amk_status.quit_dc_on) == true;
  // return (amk_rear_left_actual_values1.amk_status.quit_dc_on && amk_rear_right_actual_values1.amk_status.quit_dc_on) == true;

}

bool AmkNode::check_inv_errors()
{
  return (amk_front_left_actual_values1.amk_status.error || amk_front_right_actual_values1.amk_status.error || amk_rear_left_actual_values1.amk_status.error ||
          amk_rear_right_actual_values1.amk_status.error) == true;
  // return (amk_rear_left_actual_values1.amk_status.error ||
  //         amk_rear_right_actual_values1.amk_status.error) == true;
}

bool AmkNode::check_quit_inverter_on()
{
  return (!amk_front_left_actual_values1.amk_status.quit_inverter_on || !amk_front_right_actual_values1.amk_status.quit_inverter_on ||
          !amk_rear_left_actual_values1.amk_status.quit_inverter_on || !amk_rear_right_actual_values1.amk_status.quit_inverter_on) == false;
  // return (!amk_rear_left_actual_values1.amk_status.quit_inverter_on || !amk_rear_right_actual_values1.amk_status.quit_inverter_on) == false;
}

bool AmkNode::system_ready()
{
  return ( amk_rear_left_actual_values1.amk_status.system_ready && amk_rear_right_actual_values1.amk_status.system_ready &&
           amk_front_left_actual_values1.amk_status.system_ready && amk_front_right_actual_values1.amk_status.system_ready) == true;
}
bool AmkNode::check_inv_on()
{
  return (amk_front_left_actual_values1.amk_status.quit_inverter_on && amk_front_right_actual_values1.amk_status.quit_inverter_on &&
          amk_rear_left_actual_values1.amk_status.quit_inverter_on && amk_rear_right_actual_values1.amk_status.quit_inverter_on &&
         amk_front_left_actual_values1.amk_status.quit_dc_on && amk_front_right_actual_values1.amk_status.quit_dc_on &&
         amk_rear_left_actual_values1.amk_status.quit_dc_on && amk_rear_right_actual_values1.amk_status.quit_dc_on&&
         amk_front_left_actual_values1.amk_status.inverter_on && amk_front_right_actual_values1.amk_status.inverter_on &&
          amk_rear_left_actual_values1.amk_status.inverter_on && amk_rear_right_actual_values1.amk_status.inverter_on &&
         amk_front_left_actual_values1.amk_status.dc_on && amk_front_right_actual_values1.amk_status.dc_on &&
         amk_rear_left_actual_values1.amk_status.dc_on && amk_rear_right_actual_values1.amk_status.dc_on) == true;
  // return (
  //         amk_rear_left_actual_values1.amk_status.quit_inverter_on && amk_rear_right_actual_values1.amk_status.quit_inverter_on &&
          
  //         amk_rear_left_actual_values1.amk_status.quit_dc_on && amk_rear_right_actual_values1.amk_status.quit_dc_on&&
          
  //         amk_rear_left_actual_values1.amk_status.inverter_on && amk_rear_right_actual_values1.amk_status.inverter_on &&
          
  //         amk_rear_left_actual_values1.amk_status.dc_on && amk_rear_right_actual_values1.amk_status.dc_on) == true;
}

bool AmkNode::all_inv_on()
{
  return (amk_front_left_actual_values1.amk_status.quit_inverter_on || amk_front_right_actual_values1.amk_status.quit_inverter_on ||
          amk_rear_left_actual_values1.amk_status.quit_inverter_on || amk_rear_right_actual_values1.amk_status.quit_inverter_on ||
          amk_front_left_actual_values1.amk_status.quit_dc_on || amk_front_right_actual_values1.amk_status.quit_dc_on || 
          amk_rear_left_actual_values1.amk_status.quit_dc_on || amk_rear_right_actual_values1.amk_status.quit_dc_on||
          amk_front_left_actual_values1.amk_status.inverter_on || amk_front_right_actual_values1.amk_status.inverter_on ||
          amk_rear_left_actual_values1.amk_status.inverter_on || amk_rear_right_actual_values1.amk_status.inverter_on ||
          amk_front_left_actual_values1.amk_status.dc_on || amk_front_right_actual_values1.amk_status.dc_on || 
          amk_rear_left_actual_values1.amk_status.dc_on || amk_rear_right_actual_values1.amk_status.dc_on || rtd.state) == false;
  // return (
  //         amk_rear_left_actual_values1.amk_status.quit_inverter_on || amk_rear_right_actual_values1.amk_status.quit_inverter_on ||
          
  //         amk_rear_left_actual_values1.amk_status.quit_dc_on || amk_rear_right_actual_values1.amk_status.quit_dc_on||
          
  //         amk_rear_left_actual_values1.amk_status.inverter_on || amk_rear_right_actual_values1.amk_status.inverter_on ||
          
  //         amk_rear_left_actual_values1.amk_status.dc_on || amk_rear_right_actual_values1.amk_status.dc_on || rtd.state) == false;
}

void AmkNode::amk_state_machine_callback()
{

  for (const auto &t : transitions)
  {
    if (t.from == state && t.condition())
    {
      on_exit(t.from);
      on_enter(t.to);
      state = t.to;
      break;
    }
  }

  on_update(state);
}
void AmkNode::on_enter(StateMachine state)
{
  switch (state)
  {
  case StateMachine::IDLING:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[IDLING] Entered state");
    break;

  case StateMachine::DC_STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[DC_STARTUP] Entered state");
    amk_state_machine_watchdog->reset();
    break;

  case StateMachine::INVERTER_STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[INVERTER_STARTUP] Entered state");
    amk_state_machine_watchdog->reset();
    break;

  case StateMachine::STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[STARTUP] Entered state");
    amk_state_machine_watchdog->reset();
    break;

  case StateMachine::TORQUE_CONTROL:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[TORQUE_CONTROL] Entered state");
    break;

  case StateMachine::SWITCH_OFF:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[SWITCH_OFF] Entered state");
    rtd.state = false;
    break;

  case StateMachine::ERROR_HANDLER:
    std::cout << "[ERROR_HANDLER] Entered state\n";
    break;

  case StateMachine::ERROR_RESET:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[ERROR_RESET] Entered state");
    //FIXME: to chyba można zostawić?
    amk_front_left_setpoints.amk_control.error_reset = true;
    amk_front_right_setpoints.amk_control.error_reset = true;
    amk_rear_left_setpoints.amk_control.error_reset = true;
    amk_rear_right_setpoints.amk_control.error_reset = true;
    break;

  default:
    std::cout << "[UNKNOWN STATE] No on_enter defined on enter\n";
    break;
  }
}
void AmkNode::on_update(StateMachine state)
{
  switch (state)
  {
  case StateMachine::IDLING:
    state_machine.state = 0;
    state_machine_publisher->publish(state_machine);
    break;

  case StateMachine::DC_STARTUP:
    amk_front_left_setpoints.amk_control.dc_on = true;
    amk_front_left_setpoints.torque_positive_limit = 0;
    amk_front_left_setpoints.torque_negative_limit = 0;
    amk_front_left_setpoints.target_torque = 0;

    amk_front_right_setpoints.amk_control.dc_on = true;
    amk_front_right_setpoints.torque_positive_limit = 0;
    amk_front_right_setpoints.torque_negative_limit = 0;
    amk_front_right_setpoints.target_torque = 0;

    amk_rear_left_setpoints.amk_control.dc_on = true;
    amk_rear_left_setpoints.torque_positive_limit = 0;
    amk_rear_left_setpoints.torque_negative_limit = 0;
    amk_rear_left_setpoints.target_torque = 0;

    amk_rear_right_setpoints.amk_control.dc_on = true;
    amk_rear_right_setpoints.torque_positive_limit = 0;
    amk_rear_right_setpoints.torque_negative_limit = 0;
    amk_rear_right_setpoints.target_torque = 0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[DC_STARTUP] Waiting for bdc_on");
    rclcpp::sleep_for(10ms);
    break;

  case StateMachine::INVERTER_STARTUP:
    amk_front_left_setpoints.amk_control.inverter_on = true;
    amk_front_left_setpoints.amk_control.enable = true;

    amk_front_right_setpoints.amk_control.inverter_on = true;
    amk_front_right_setpoints.amk_control.enable = true;

    amk_rear_left_setpoints.amk_control.inverter_on = true;
    amk_rear_left_setpoints.amk_control.enable = true;

    amk_rear_right_setpoints.amk_control.inverter_on = true;
    amk_rear_right_setpoints.amk_control.enable = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[INVERTER_STARTUP] Waiting for inverter enable");
    rclcpp::sleep_for(10ms);
    break;

  case StateMachine::STARTUP:
    amk_front_left_setpoints.amk_control.enable = true;
    amk_front_right_setpoints.amk_control.enable = true;
    amk_rear_left_setpoints.amk_control.enable = true;
    amk_rear_right_setpoints.amk_control.enable = true;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[STARTUP] Waiting for bquit inverter on");
    rclcpp::sleep_for(10ms);
    break;

  case StateMachine::TORQUE_CONTROL:
    amk_front_left_setpoints.torque_positive_limit = 2000;
    amk_front_left_setpoints.torque_negative_limit = -2000;
    amk_front_left_setpoints.target_torque = setpoints.front_left.torque;

    amk_front_right_setpoints.torque_positive_limit = 2000;
    amk_front_right_setpoints.torque_negative_limit = -2000;
    amk_front_right_setpoints.target_torque = setpoints.front_right.torque;

    amk_rear_left_setpoints.torque_positive_limit = 2000;
    amk_rear_left_setpoints.torque_negative_limit = -2000;
    amk_rear_left_setpoints.target_torque = setpoints.rear_left.torque;

    amk_rear_right_setpoints.torque_positive_limit = 2000;
    amk_rear_right_setpoints.torque_negative_limit = -2000;
    amk_rear_right_setpoints.target_torque = -setpoints.rear_right.torque;
    // if (amk_rear_left_actual_values1.actual_velocity > 20000 || amk_rear_right_actual_values1.actual_velocity > 20000){
    if (amk_front_left_actual_values1.actual_velocity > 20000 || amk_front_right_actual_values1.actual_velocity > 20000 || amk_rear_left_actual_values1.actual_velocity > 20000 || amk_rear_right_actual_values1.actual_velocity > 20000){
      amk_front_left_setpoints.target_torque = 0;
      amk_front_right_setpoints.target_torque = 0;
      amk_rear_left_setpoints.target_torque = 0;
      amk_rear_right_setpoints.target_torque = 0;
    }
    break;

  case StateMachine::SWITCH_OFF:
    amk_front_left_setpoints.amk_control.inverter_on = false;
    amk_front_left_setpoints.amk_control.enable = false;
    amk_front_left_setpoints.amk_control.dc_on = false;
    amk_front_left_setpoints.torque_positive_limit = 0;
    amk_front_left_setpoints.torque_negative_limit = 0;
    amk_front_left_setpoints.target_torque = 0;

    amk_front_right_setpoints.amk_control.inverter_on = false;
    amk_front_right_setpoints.amk_control.enable = false;
    amk_front_right_setpoints.amk_control.dc_on = false;
    amk_front_right_setpoints.torque_positive_limit = 0;
    amk_front_right_setpoints.torque_negative_limit = 0;
    amk_front_right_setpoints.target_torque = 0;

    amk_rear_left_setpoints.amk_control.inverter_on = false;
    amk_rear_left_setpoints.amk_control.enable = false;
    amk_rear_left_setpoints.amk_control.dc_on = false;
    amk_rear_left_setpoints.torque_positive_limit = 0;
    amk_rear_left_setpoints.torque_negative_limit = 0;
    amk_rear_left_setpoints.target_torque = 0;

    amk_rear_right_setpoints.amk_control.inverter_on = false;
    amk_rear_right_setpoints.amk_control.enable = false;
    amk_rear_right_setpoints.amk_control.dc_on = false;
    amk_rear_right_setpoints.torque_positive_limit = 0;
    amk_rear_right_setpoints.torque_negative_limit = 0;
    amk_rear_right_setpoints.target_torque = 0;
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[SWITCH_OFF] Waiting for inverter switch-off");
    state_machine.state = 1;
    state_machine_publisher->publish(state_machine);
    rclcpp::sleep_for(5ms);
    break;

  case StateMachine::ERROR_HANDLER:
    std::cout << "[ERROR_HANDLER] Performing actions...\n";
    break;

  case StateMachine::ERROR_RESET:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[ERROR_RESET]Waiting for error reset");
    rclcpp::sleep_for(10ms);
    break;

  case StateMachine::UNDEFINED:
    std::cout << "[UNDEFINED] Performing actions...\n";
    break;

  default:
    std::cout << "[UNKNOWN STATE] No actions defined\n";
    break;
  }
}

void AmkNode::on_exit(StateMachine state)
{
  switch (state)
  {
  case StateMachine::IDLING:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[IDLING] exit state");
    break;

  case StateMachine::DC_STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[DC_STARTUP] bdc on");
    break;
  case StateMachine::INVERTER_STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[INVERTER_STARTUP] inverters enabled");
    amk_state_machine_watchdog->cancel();
    break;

  case StateMachine::STARTUP:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[STARTUP] Inverters On");
    rclcpp::sleep_for(10ms);
    amk_state_machine_watchdog->cancel();
    break;

  case StateMachine::TORQUE_CONTROL:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[TORQUE_CONTROL] exit state");
    break;

  case StateMachine::SWITCH_OFF:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[SWITCH_OFF] exit state");
    break;

  case StateMachine::ERROR_HANDLER:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[ERROR_HANDLER] exit state");
    break;

  case StateMachine::ERROR_RESET:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[ERROR_RESET] Error recovered");
    amk_front_left_setpoints.amk_control.error_reset = false;
    amk_front_right_setpoints.amk_control.error_reset = false;
    amk_rear_left_setpoints.amk_control.error_reset = false;
    amk_rear_right_setpoints.amk_control.error_reset = false;
    break;

  case StateMachine::UNDEFINED:
    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "[UNDEFINED] exit state");
    break;

  default:
    std::cout << "[UNKNOWN STATE] No actions defined\n";
    break;
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<AmkNode>());
  rclcpp::shutdown();
}