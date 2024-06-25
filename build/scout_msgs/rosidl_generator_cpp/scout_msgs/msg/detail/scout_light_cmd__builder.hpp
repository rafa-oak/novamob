// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__BUILDER_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scout_msgs/msg/detail/scout_light_cmd__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scout_msgs
{

namespace msg
{

namespace builder
{

class Init_ScoutLightCmd_rear_custom_value
{
public:
  explicit Init_ScoutLightCmd_rear_custom_value(::scout_msgs::msg::ScoutLightCmd & msg)
  : msg_(msg)
  {}
  ::scout_msgs::msg::ScoutLightCmd rear_custom_value(::scout_msgs::msg::ScoutLightCmd::_rear_custom_value_type arg)
  {
    msg_.rear_custom_value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightCmd msg_;
};

class Init_ScoutLightCmd_rear_mode
{
public:
  explicit Init_ScoutLightCmd_rear_mode(::scout_msgs::msg::ScoutLightCmd & msg)
  : msg_(msg)
  {}
  Init_ScoutLightCmd_rear_custom_value rear_mode(::scout_msgs::msg::ScoutLightCmd::_rear_mode_type arg)
  {
    msg_.rear_mode = std::move(arg);
    return Init_ScoutLightCmd_rear_custom_value(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightCmd msg_;
};

class Init_ScoutLightCmd_front_custom_value
{
public:
  explicit Init_ScoutLightCmd_front_custom_value(::scout_msgs::msg::ScoutLightCmd & msg)
  : msg_(msg)
  {}
  Init_ScoutLightCmd_rear_mode front_custom_value(::scout_msgs::msg::ScoutLightCmd::_front_custom_value_type arg)
  {
    msg_.front_custom_value = std::move(arg);
    return Init_ScoutLightCmd_rear_mode(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightCmd msg_;
};

class Init_ScoutLightCmd_front_mode
{
public:
  explicit Init_ScoutLightCmd_front_mode(::scout_msgs::msg::ScoutLightCmd & msg)
  : msg_(msg)
  {}
  Init_ScoutLightCmd_front_custom_value front_mode(::scout_msgs::msg::ScoutLightCmd::_front_mode_type arg)
  {
    msg_.front_mode = std::move(arg);
    return Init_ScoutLightCmd_front_custom_value(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightCmd msg_;
};

class Init_ScoutLightCmd_cmd_ctrl_allowed
{
public:
  Init_ScoutLightCmd_cmd_ctrl_allowed()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScoutLightCmd_front_mode cmd_ctrl_allowed(::scout_msgs::msg::ScoutLightCmd::_cmd_ctrl_allowed_type arg)
  {
    msg_.cmd_ctrl_allowed = std::move(arg);
    return Init_ScoutLightCmd_front_mode(msg_);
  }

private:
  ::scout_msgs::msg::ScoutLightCmd msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scout_msgs::msg::ScoutLightCmd>()
{
  return scout_msgs::msg::builder::Init_ScoutLightCmd_cmd_ctrl_allowed();
}

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__BUILDER_HPP_
