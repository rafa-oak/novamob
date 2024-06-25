// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__BUILDER_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scout_msgs/msg/detail/scout_actuator_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scout_msgs
{

namespace msg
{

namespace builder
{

class Init_ScoutActuatorState_driver_state
{
public:
  explicit Init_ScoutActuatorState_driver_state(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  ::scout_msgs::msg::ScoutActuatorState driver_state(::scout_msgs::msg::ScoutActuatorState::_driver_state_type arg)
  {
    msg_.driver_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_motor_temperature
{
public:
  explicit Init_ScoutActuatorState_motor_temperature(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_driver_state motor_temperature(::scout_msgs::msg::ScoutActuatorState::_motor_temperature_type arg)
  {
    msg_.motor_temperature = std::move(arg);
    return Init_ScoutActuatorState_driver_state(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_driver_temperature
{
public:
  explicit Init_ScoutActuatorState_driver_temperature(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_motor_temperature driver_temperature(::scout_msgs::msg::ScoutActuatorState::_driver_temperature_type arg)
  {
    msg_.driver_temperature = std::move(arg);
    return Init_ScoutActuatorState_motor_temperature(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_driver_voltage
{
public:
  explicit Init_ScoutActuatorState_driver_voltage(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_driver_temperature driver_voltage(::scout_msgs::msg::ScoutActuatorState::_driver_voltage_type arg)
  {
    msg_.driver_voltage = std::move(arg);
    return Init_ScoutActuatorState_driver_temperature(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_pulse_count
{
public:
  explicit Init_ScoutActuatorState_pulse_count(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_driver_voltage pulse_count(::scout_msgs::msg::ScoutActuatorState::_pulse_count_type arg)
  {
    msg_.pulse_count = std::move(arg);
    return Init_ScoutActuatorState_driver_voltage(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_current
{
public:
  explicit Init_ScoutActuatorState_current(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_pulse_count current(::scout_msgs::msg::ScoutActuatorState::_current_type arg)
  {
    msg_.current = std::move(arg);
    return Init_ScoutActuatorState_pulse_count(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_rpm
{
public:
  explicit Init_ScoutActuatorState_rpm(::scout_msgs::msg::ScoutActuatorState & msg)
  : msg_(msg)
  {}
  Init_ScoutActuatorState_current rpm(::scout_msgs::msg::ScoutActuatorState::_rpm_type arg)
  {
    msg_.rpm = std::move(arg);
    return Init_ScoutActuatorState_current(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

class Init_ScoutActuatorState_motor_id
{
public:
  Init_ScoutActuatorState_motor_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScoutActuatorState_rpm motor_id(::scout_msgs::msg::ScoutActuatorState::_motor_id_type arg)
  {
    msg_.motor_id = std::move(arg);
    return Init_ScoutActuatorState_rpm(msg_);
  }

private:
  ::scout_msgs::msg::ScoutActuatorState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scout_msgs::msg::ScoutActuatorState>()
{
  return scout_msgs::msg::builder::Init_ScoutActuatorState_motor_id();
}

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__BUILDER_HPP_
