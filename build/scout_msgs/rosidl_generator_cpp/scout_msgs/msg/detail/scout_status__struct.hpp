// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"
// Member 'actuator_states'
#include "scout_msgs/msg/detail/scout_actuator_state__struct.hpp"
// Member 'front_light_state'
// Member 'rear_light_state'
#include "scout_msgs/msg/detail/scout_light_state__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__scout_msgs__msg__ScoutStatus __attribute__((deprecated))
#else
# define DEPRECATED__scout_msgs__msg__ScoutStatus __declspec(deprecated)
#endif

namespace scout_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScoutStatus_
{
  using Type = ScoutStatus_<ContainerAllocator>;

  explicit ScoutStatus_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init),
    front_light_state(_init),
    rear_light_state(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
      this->vehicle_state = 0;
      this->control_mode = 0;
      this->error_code = 0;
      this->battery_voltage = 0.0;
      this->actuator_states.fill(scout_msgs::msg::ScoutActuatorState_<ContainerAllocator>{_init});
      this->light_control_enabled = false;
    }
  }

  explicit ScoutStatus_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init),
    actuator_states(_alloc),
    front_light_state(_alloc, _init),
    rear_light_state(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->linear_velocity = 0.0;
      this->angular_velocity = 0.0;
      this->vehicle_state = 0;
      this->control_mode = 0;
      this->error_code = 0;
      this->battery_voltage = 0.0;
      this->actuator_states.fill(scout_msgs::msg::ScoutActuatorState_<ContainerAllocator>{_alloc, _init});
      this->light_control_enabled = false;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _linear_velocity_type =
    double;
  _linear_velocity_type linear_velocity;
  using _angular_velocity_type =
    double;
  _angular_velocity_type angular_velocity;
  using _vehicle_state_type =
    uint8_t;
  _vehicle_state_type vehicle_state;
  using _control_mode_type =
    uint8_t;
  _control_mode_type control_mode;
  using _error_code_type =
    uint16_t;
  _error_code_type error_code;
  using _battery_voltage_type =
    double;
  _battery_voltage_type battery_voltage;
  using _actuator_states_type =
    std::array<scout_msgs::msg::ScoutActuatorState_<ContainerAllocator>, 4>;
  _actuator_states_type actuator_states;
  using _light_control_enabled_type =
    bool;
  _light_control_enabled_type light_control_enabled;
  using _front_light_state_type =
    scout_msgs::msg::ScoutLightState_<ContainerAllocator>;
  _front_light_state_type front_light_state;
  using _rear_light_state_type =
    scout_msgs::msg::ScoutLightState_<ContainerAllocator>;
  _rear_light_state_type rear_light_state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__linear_velocity(
    const double & _arg)
  {
    this->linear_velocity = _arg;
    return *this;
  }
  Type & set__angular_velocity(
    const double & _arg)
  {
    this->angular_velocity = _arg;
    return *this;
  }
  Type & set__vehicle_state(
    const uint8_t & _arg)
  {
    this->vehicle_state = _arg;
    return *this;
  }
  Type & set__control_mode(
    const uint8_t & _arg)
  {
    this->control_mode = _arg;
    return *this;
  }
  Type & set__error_code(
    const uint16_t & _arg)
  {
    this->error_code = _arg;
    return *this;
  }
  Type & set__battery_voltage(
    const double & _arg)
  {
    this->battery_voltage = _arg;
    return *this;
  }
  Type & set__actuator_states(
    const std::array<scout_msgs::msg::ScoutActuatorState_<ContainerAllocator>, 4> & _arg)
  {
    this->actuator_states = _arg;
    return *this;
  }
  Type & set__light_control_enabled(
    const bool & _arg)
  {
    this->light_control_enabled = _arg;
    return *this;
  }
  Type & set__front_light_state(
    const scout_msgs::msg::ScoutLightState_<ContainerAllocator> & _arg)
  {
    this->front_light_state = _arg;
    return *this;
  }
  Type & set__rear_light_state(
    const scout_msgs::msg::ScoutLightState_<ContainerAllocator> & _arg)
  {
    this->rear_light_state = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int8_t MOTOR_ID_FRONT_RIGHT =
    0;
  static constexpr int8_t MOTOR_ID_FRONT_LEFT =
    1;
  static constexpr int8_t MOTOR_ID_REAR_RIGHT =
    2;
  static constexpr int8_t MOTOR_ID_REAR_LEFT =
    3;
  static constexpr int8_t LIGHT_ID_FRONT =
    0;
  static constexpr int8_t LIGHT_ID_REAR =
    1;

  // pointer types
  using RawPtr =
    scout_msgs::msg::ScoutStatus_<ContainerAllocator> *;
  using ConstRawPtr =
    const scout_msgs::msg::ScoutStatus_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scout_msgs::msg::ScoutStatus_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scout_msgs::msg::ScoutStatus_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scout_msgs__msg__ScoutStatus
    std::shared_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scout_msgs__msg__ScoutStatus
    std::shared_ptr<scout_msgs::msg::ScoutStatus_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScoutStatus_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->linear_velocity != other.linear_velocity) {
      return false;
    }
    if (this->angular_velocity != other.angular_velocity) {
      return false;
    }
    if (this->vehicle_state != other.vehicle_state) {
      return false;
    }
    if (this->control_mode != other.control_mode) {
      return false;
    }
    if (this->error_code != other.error_code) {
      return false;
    }
    if (this->battery_voltage != other.battery_voltage) {
      return false;
    }
    if (this->actuator_states != other.actuator_states) {
      return false;
    }
    if (this->light_control_enabled != other.light_control_enabled) {
      return false;
    }
    if (this->front_light_state != other.front_light_state) {
      return false;
    }
    if (this->rear_light_state != other.rear_light_state) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScoutStatus_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScoutStatus_

// alias to use template instance with default allocator
using ScoutStatus =
  scout_msgs::msg::ScoutStatus_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::MOTOR_ID_FRONT_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::MOTOR_ID_FRONT_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::MOTOR_ID_REAR_RIGHT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::MOTOR_ID_REAR_LEFT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::LIGHT_ID_FRONT;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int8_t ScoutStatus_<ContainerAllocator>::LIGHT_ID_REAR;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_HPP_
