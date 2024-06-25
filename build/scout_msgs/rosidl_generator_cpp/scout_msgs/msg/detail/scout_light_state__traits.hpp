// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scout_msgs:msg/ScoutLightState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__TRAITS_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scout_msgs/msg/detail/scout_light_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace scout_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScoutLightState & msg,
  std::ostream & out)
{
  out << "{";
  // member: mode
  {
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << ", ";
  }

  // member: custom_value
  {
    out << "custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScoutLightState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: mode
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mode: ";
    rosidl_generator_traits::value_to_yaml(msg.mode, out);
    out << "\n";
  }

  // member: custom_value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "custom_value: ";
    rosidl_generator_traits::value_to_yaml(msg.custom_value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScoutLightState & msg, bool use_flow_style = false)
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
  const scout_msgs::msg::ScoutLightState & msg,
  std::ostream & out, size_t indentation = 0)
{
  scout_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scout_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scout_msgs::msg::ScoutLightState & msg)
{
  return scout_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scout_msgs::msg::ScoutLightState>()
{
  return "scout_msgs::msg::ScoutLightState";
}

template<>
inline const char * name<scout_msgs::msg::ScoutLightState>()
{
  return "scout_msgs/msg/ScoutLightState";
}

template<>
struct has_fixed_size<scout_msgs::msg::ScoutLightState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scout_msgs::msg::ScoutLightState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scout_msgs::msg::ScoutLightState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__TRAITS_HPP_
