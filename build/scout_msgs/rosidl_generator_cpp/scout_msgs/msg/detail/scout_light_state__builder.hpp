// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scout_msgs:msg/ScoutLightState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__BUILDER_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scout_msgs/msg/detail/scout_light_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scout_msgs
{

namespace msg
{

namespace builder
{

class Init_ScoutLightState_custom_value
{
public:
  explicit Init_ScoutLightState_custom_value(::scout_msgs::msg::ScoutLightState & msg)
  : msg_(msg)
  {}
  ::scout_msgs::msg::ScoutLightState custom_value(::scout_msgs::msg::ScoutLightState::_custom_value_type arg)
  {
    msg_.custom_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightState msg_;
};

class Init_ScoutLightState_mode
{
public:
  Init_ScoutLightState_mode()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScoutLightState_custom_value mode(::scout_msgs::msg::ScoutLightState::_mode_type arg)
  {
    msg_.mode = std::move(arg);
    return Init_ScoutLightState_custom_value(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scout_msgs::msg::ScoutLightState>()
{
  return scout_msgs::msg::builder::Init_ScoutLightState_mode();
}

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__BUILDER_HPP_
