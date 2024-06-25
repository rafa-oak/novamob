// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__BUILDER_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scout_msgs/msg/detail/scout_status__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scout_msgs
{

namespace msg
{

namespace builder
{

class Init_ScoutStatus_rear_light_state
{
public:
  explicit Init_ScoutStatus_rear_light_state(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  ::scout_msgs::msg::ScoutStatus rear_light_state(::scout_msgs::msg::ScoutStatus::_rear_light_state_type arg)
  {
    msg_.rear_light_state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_front_light_state
{
public:
  explicit Init_ScoutStatus_front_light_state(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_rear_light_state front_light_state(::scout_msgs::msg::ScoutStatus::_front_light_state_type arg)
  {
    msg_.front_light_state = std::move(arg);
    return Init_ScoutStatus_rear_light_state(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_light_control_enabled
{
public:
  explicit Init_ScoutStatus_light_control_enabled(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_front_light_state light_control_enabled(::scout_msgs::msg::ScoutStatus::_light_control_enabled_type arg)
  {
    msg_.light_control_enabled = std::move(arg);
    return Init_ScoutStatus_front_light_state(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_actuator_states
{
public:
  explicit Init_ScoutStatus_actuator_states(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_light_control_enabled actuator_states(::scout_msgs::msg::ScoutStatus::_actuator_states_type arg)
  {
    msg_.actuator_states = std::move(arg);
    return Init_ScoutStatus_light_control_enabled(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_battery_voltage
{
public:
  explicit Init_ScoutStatus_battery_voltage(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_actuator_states battery_voltage(::scout_msgs::msg::ScoutStatus::_battery_voltage_type arg)
  {
    msg_.battery_voltage = std::move(arg);
    return Init_ScoutStatus_actuator_states(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_error_code
{
public:
  explicit Init_ScoutStatus_error_code(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_battery_voltage error_code(::scout_msgs::msg::ScoutStatus::_error_code_type arg)
  {
    msg_.error_code = std::move(arg);
    return Init_ScoutStatus_battery_voltage(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_control_mode
{
public:
  explicit Init_ScoutStatus_control_mode(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_error_code control_mode(::scout_msgs::msg::ScoutStatus::_control_mode_type arg)
  {
    msg_.control_mode = std::move(arg);
    return Init_ScoutStatus_error_code(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_vehicle_state
{
public:
  explicit Init_ScoutStatus_vehicle_state(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_control_mode vehicle_state(::scout_msgs::msg::ScoutStatus::_vehicle_state_type arg)
  {
    msg_.vehicle_state = std::move(arg);
    return Init_ScoutStatus_control_mode(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_angular_velocity
{
public:
  explicit Init_ScoutStatus_angular_velocity(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_vehicle_state angular_velocity(::scout_msgs::msg::ScoutStatus::_angular_velocity_type arg)
  {
    msg_.angular_velocity = std::move(arg);
    return Init_ScoutStatus_vehicle_state(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_linear_velocity
{
public:
  explicit Init_ScoutStatus_linear_velocity(::scout_msgs::msg::ScoutStatus & msg)
  : msg_(msg)
  {}
  Init_ScoutStatus_angular_velocity linear_velocity(::scout_msgs::msg::ScoutStatus::_linear_velocity_type arg)
  {
    msg_.linear_velocity = std::move(arg);
    return Init_ScoutStatus_angular_velocity(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

class Init_ScoutStatus_header
{
public:
  Init_ScoutStatus_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScoutStatus_linear_velocity header(::scout_msgs::msg::ScoutStatus::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_ScoutStatus_linear_velocity(msg_);
  }

private:
  ::scout_msgs::msg::ScoutStatus msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scout_msgs::msg::ScoutStatus>()
{
  return scout_msgs::msg::builder::Init_ScoutStatus_header();
}

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__BUILDER_HPP_
