// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__TRAITS_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scout_msgs/msg/detail/scout_light_cmd__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace scout_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScoutLightCmd & msg,
  std::ostream & out)
{
  out << "{";
  // member: cmd_ctrl_allowed
  {
    out << "cmd_ctrl_allowed: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_ctrl_allowed, out);
    out << ", ";
  }

  // member: front_mode
  {
    out << "front_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.front_mode, out);
    out << ", ";
  }

  // member: front_custom_value
  {
    out << "front_custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.front_custom_value, out);
    out << ", ";
  }

  // member: rear_mode
  {
    out << "rear_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_mode, out);
    out << ", ";
  }

  // member: rear_custom_value
  {
    out << "rear_custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_custom_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScoutLightCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: cmd_ctrl_allowed
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "cmd_ctrl_allowed: ";
    rosidl_generator_traits::value_to_yaml(msg.cmd_ctrl_allowed, out);
    out << "\n";
  }

  // member: front_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.front_mode, out);
    out << "\n";
  }

  // member: front_custom_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "front_custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.front_custom_value, out);
    out << "\n";
  }

  // member: rear_mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_mode: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_mode, out);
    out << "\n";
  }

  // member: rear_custom_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rear_custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.rear_custom_value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScoutLightCmd & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace scout_msgs

namespace rosidl_generator_traits
{

[[deprecated("use scout_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const scout_msgs::msg::ScoutLightCmd & msg,
  std::ostream & out, size_t indentation = 0)
{
  scout_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scout_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scout_msgs::msg::ScoutLightCmd & msg)
{
  return scout_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scout_msgs::msg::ScoutLightCmd>()
{
  return "scout_msgs::msg::ScoutLightCmd";
}

template<>
inline const char * name<scout_msgs::msg::ScoutLightCmd>()
{
  return "scout_msgs/msg/ScoutLightCmd";
}

template<>
struct has_fixed_size<scout_msgs::msg::ScoutLightCmd>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scout_msgs::msg::ScoutLightCmd>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scout_msgs::msg::ScoutLightCmd>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__TRAITS_HPP_
