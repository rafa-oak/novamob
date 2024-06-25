// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__TRAITS_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "scout_msgs/msg/detail/scout_actuator_state__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace scout_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const ScoutActuatorState & msg,
  std::ostream & out)
{
  out << "{";
  // member: motor_id
  {
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << ", ";
  }

  // member: rpm
  {
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << ", ";
  }

  // member: current
  {
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << ", ";
  }

  // member: pulse_count
  {
    out << "pulse_count: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_count, out);
    out << ", ";
  }

  // member: driver_voltage
  {
    out << "driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_voltage, out);
    out << ", ";
  }

  // member: driver_temperature
  {
    out << "driver_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_temperature, out);
    out << ", ";
  }

  // member: motor_temperature
  {
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << ", ";
  }

  // member: driver_state
  {
    out << "driver_state: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_state, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const ScoutActuatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: motor_id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_id: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_id, out);
    out << "\n";
  }

  // member: rpm
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "rpm: ";
    rosidl_generator_traits::value_to_yaml(msg.rpm, out);
    out << "\n";
  }

  // member: current
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "current: ";
    rosidl_generator_traits::value_to_yaml(msg.current, out);
    out << "\n";
  }

  // member: pulse_count
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "pulse_count: ";
    rosidl_generator_traits::value_to_yaml(msg.pulse_count, out);
    out << "\n";
  }

  // member: driver_voltage
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_voltage: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_voltage, out);
    out << "\n";
  }

  // member: driver_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_temperature, out);
    out << "\n";
  }

  // member: motor_temperature
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "motor_temperature: ";
    rosidl_generator_traits::value_to_yaml(msg.motor_temperature, out);
    out << "\n";
  }

  // member: driver_state
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "driver_state: ";
    rosidl_generator_traits::value_to_yaml(msg.driver_state, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const ScoutActuatorState & msg, bool use_flow_style = false)
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
  const scout_msgs::msg::ScoutActuatorState & msg,
  std::ostream & out, size_t indentation = 0)
{
  scout_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use scout_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const scout_msgs::msg::ScoutActuatorState & msg)
{
  return scout_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<scout_msgs::msg::ScoutActuatorState>()
{
  return "scout_msgs::msg::ScoutActuatorState";
}

template<>
inline const char * name<scout_msgs::msg::ScoutActuatorState>()
{
  return "scout_msgs/msg/ScoutActuatorState";
}

template<>
struct has_fixed_size<scout_msgs::msg::ScoutActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<scout_msgs::msg::ScoutActuatorState>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<scout_msgs::msg::ScoutActuatorState>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__TRAITS_HPP_
