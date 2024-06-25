// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scout_msgs:msg/ScoutLightState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ScoutLightState in the package scout_msgs.
typedef struct scout_msgs__msg__ScoutLightState
{
  uint8_t mode;
  uint8_t custom_value;
} scout_msgs__msg__ScoutLightState;

// Struct for a sequence of scout_msgs__msg__ScoutLightState.
typedef struct scout_msgs__msg__ScoutLightState__Sequence
{
  scout_msgs__msg__ScoutLightState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scout_msgs__msg__ScoutLightState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_STATE__STRUCT_H_
