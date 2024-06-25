// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from scout_msgs:msg/ScoutRCState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__STRUCT_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/ScoutRCState in the package scout_msgs.
typedef struct scout_msgs__msg__ScoutRCState
{
  uint8_t swa;
  uint8_t swb;
  uint8_t swc;
  uint8_t swd;
  int8_t stick_right_v;
  int8_t stick_right_h;
  int8_t stick_left_v;
  int8_t stick_left_h;
  int8_t var_a;
} scout_msgs__msg__ScoutRCState;

// Struct for a sequence of scout_msgs__msg__ScoutRCState.
typedef struct scout_msgs__msg__ScoutRCState__Sequence
{
  scout_msgs__msg__ScoutRCState * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} scout_msgs__msg__ScoutRCState__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_RC_STATE__STRUCT_H_
