// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from scout_msgs:msg/ScoutRCState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__BUILDER_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "scout_msgs/msg/detail/scout_rc_state__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace scout_msgs
{

namespace msg
{

namespace builder
{

class Init_ScoutRCState_var_a
{
public:
  explicit Init_ScoutRCState_var_a(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  ::scout_msgs::msg::ScoutRCState var_a(::scout_msgs::msg::ScoutRCState::_var_a_type arg)
  {
    msg_.var_a = std::move(arg);
    return std::move(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_stick_left_h
{
public:
  explicit Init_ScoutRCState_stick_left_h(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_var_a stick_left_h(::scout_msgs::msg::ScoutRCState::_stick_left_h_type arg)
  {
    msg_.stick_left_h = std::move(arg);
    return Init_ScoutRCState_var_a(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_stick_left_v
{
public:
  explicit Init_ScoutRCState_stick_left_v(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_stick_left_h stick_left_v(::scout_msgs::msg::ScoutRCState::_stick_left_v_type arg)
  {
    msg_.stick_left_v = std::move(arg);
    return Init_ScoutRCState_stick_left_h(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_stick_right_h
{
public:
  explicit Init_ScoutRCState_stick_right_h(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_stick_left_v stick_right_h(::scout_msgs::msg::ScoutRCState::_stick_right_h_type arg)
  {
    msg_.stick_right_h = std::move(arg);
    return Init_ScoutRCState_stick_left_v(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_stick_right_v
{
public:
  explicit Init_ScoutRCState_stick_right_v(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_stick_right_h stick_right_v(::scout_msgs::msg::ScoutRCState::_stick_right_v_type arg)
  {
    msg_.stick_right_v = std::move(arg);
    return Init_ScoutRCState_stick_right_h(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_swd
{
public:
  explicit Init_ScoutRCState_swd(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_stick_right_v swd(::scout_msgs::msg::ScoutRCState::_swd_type arg)
  {
    msg_.swd = std::move(arg);
    return Init_ScoutRCState_stick_right_v(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_swc
{
public:
  explicit Init_ScoutRCState_swc(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_swd swc(::scout_msgs::msg::ScoutRCState::_swc_type arg)
  {
    msg_.swc = std::move(arg);
    return Init_ScoutRCState_swd(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_swb
{
public:
  explicit Init_ScoutRCState_swb(::scout_msgs::msg::ScoutRCState & msg)
  : msg_(msg)
  {}
  Init_ScoutRCState_swc swb(::scout_msgs::msg::ScoutRCState::_swb_type arg)
  {
    msg_.swb = std::move(arg);
    return Init_ScoutRCState_swc(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

class Init_ScoutRCState_swa
{
public:
  Init_ScoutRCState_swa()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_ScoutRCState_swb swa(::scout_msgs::msg::ScoutRCState::_swa_type arg)
  {
    msg_.swa = std::move(arg);
    return Init_ScoutRCState_swb(msg_);
  }

private:
  ::scout_msgs::msg::ScoutRCState msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::scout_msgs::msg::ScoutRCState>()
{
  return scout_msgs::msg::builder::Init_ScoutRCState_swa();
}

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__BUILDER_HPP_
