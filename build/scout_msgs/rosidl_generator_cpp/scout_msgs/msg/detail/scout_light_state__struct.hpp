// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from scout_msgs:msg/ScoutLightState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_HPP_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__scout_msgs__msg__ScoutLightState __attribute__((deprecated))
#else
# define DEPRECATED__scout_msgs__msg__ScoutLightState __declspec(deprecated)
#endif

namespace scout_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct ScoutLightState_
{
  using Type = ScoutLightState_<ContainerAllocator>;

  explicit ScoutLightState_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->custom_value = 0;
    }
  }

  explicit ScoutLightState_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->mode = 0;
      this->custom_value = 0;
    }
  }

  // field types and members
  using _mode_type =
    uint8_t;
  _mode_type mode;
  using _custom_value_type =
    uint8_t;
  _custom_value_type custom_value;

  // setters for named parameter idiom
  Type & set__mode(
    const uint8_t & _arg)
  {
    this->mode = _arg;
    return *this;
  }
  Type & set__custom_value(
    const uint8_t & _arg)
  {
    this->custom_value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    scout_msgs::msg::ScoutLightState_<ContainerAllocator> *;
  using ConstRawPtr =
    const scout_msgs::msg::ScoutLightState_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      scout_msgs::msg::ScoutLightState_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      scout_msgs::msg::ScoutLightState_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__scout_msgs__msg__ScoutLightState
    std::shared_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__scout_msgs__msg__ScoutLightState
    std::shared_ptr<scout_msgs::msg::ScoutLightState_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const ScoutLightState_ & other) const
  {
    if (this->mode != other.mode) {
      return false;
    }
    if (this->custom_value != other.custom_value) {
      return false;
    }
    return true;
  }
  bool operator!=(const ScoutLightState_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct ScoutLightState_

// alias to use template instance with default allocator
using ScoutLightState =
  scout_msgs::msg::ScoutLightState_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace scout_msgs

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_HPP_
