// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'MOTOR_ID_FRONT_RIGHT'.
enum
{
  scout_msgs__msg__ScoutStatus__MOTOR_ID_FRONT_RIGHT = 0
};

/// Constant 'MOTOR_ID_FRONT_LEFT'.
enum
{
  scout_msgs__msg__ScoutStatus__MOTOR_ID_FRONT_LEFT = 1
};

/// Constant 'MOTOR_ID_REAR_RIGHT'.
enum
{
  scout_msgs__msg__ScoutStatus__MOTOR_ID_REAR_RIGHT = 2
};

/// Constant 'MOTOR_ID_REAR_LEFT'.
enum
{
  scout_msgs__msg__ScoutStatus__MOTOR_ID_REAR_LEFT = 3
};

/// Constant 'LIGHT_ID_FRONT'.
enum
{
  scout_msgs__msg__ScoutStatus__LIGHT_ID_FRONT = 0
};

/// Constant 'LIGHT_ID_REAR'.
enum
{
  scout_msgs__msg__ScoutStatus__LIGHT_ID_REAR = 1
};

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'actuator_states'
#include "scout_msgs/msg/detail/scout_actuator_state__struct.h"
// Member 'front_light_state'
// Member 'rear_light_state'
#include "scout_msgs/msg/detail/scout_light_state__struct.h"

/// Struct defined in msg/ScoutStatus in the package scout_msgs.
typedef struct scout_msgs__msg__ScoutStatus
{
  std_msgs__msg__Header header;
  /// motion state
  double linear_velocity;
  double angular_velocity;
  /// base state
  uint8_t vehicle_state;
  uint8_t control_mode;
  uint16_t error_code;
  double battery_voltage;
  /// motor state
  scout_msgs__msg__ScoutActuatorState actuator_states[4];
  /// light state
  bool light_control_enabled;
  scout_msgs__msg__ScoutLightState front_light_state;
  scout_msgs__msg__ScoutLightState rear_light_state;
} scout_msgs__msg__ScoutStatus;

// Struct for a sequence of scout_msgs__msg__ScoutStatus.
typedef struct scout_msgs__msg__ScoutStatus__Sequence
{
  scout_msgs__msg__ScoutStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scout_msgs__msg__ScoutStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__STRUCT_H_
