// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__STRUCT_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'LIGHT_CONST_OFF'.
enum
{
  scout_msgs__msg__ScoutLightCmd__LIGHT_CONST_OFF = 0
};

/// Constant 'LIGHT_CONST_ON'.
enum
{
  scout_msgs__msg__ScoutLightCmd__LIGHT_CONST_ON = 1
};

/// Constant 'LIGHT_BREATH'.
enum
{
  scout_msgs__msg__ScoutLightCmd__LIGHT_BREATH = 2
};

/// Constant 'LIGHT_CUSTOM'.
enum
{
  scout_msgs__msg__ScoutLightCmd__LIGHT_CUSTOM = 3
};

/// Struct defined in msg/ScoutLightCmd in the package scout_msgs.
typedef struct scout_msgs__msg__ScoutLightCmd
{
  bool cmd_ctrl_allowed;
  uint8_t front_mode;
  uint8_t front_custom_value;
  uint8_t rear_mode;
  uint8_t rear_custom_value;
} scout_msgs__msg__ScoutLightCmd;

// Struct for a sequence of scout_msgs__msg__ScoutLightCmd.
typedef struct scout_msgs__msg__ScoutLightCmd__Sequence
{
  scout_msgs__msg__ScoutLightCmd * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scout_msgs__msg__ScoutLightCmd__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__STRUCT_H_
