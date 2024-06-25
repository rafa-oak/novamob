// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__FUNCTIONS_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "scout_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "scout_msgs/msg/detail/scout_actuator_state__struct.h"

/// Initialize msg/ScoutActuatorState message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * scout_msgs__msg__ScoutActuatorState
 * )) before or use
 * scout_msgs__msg__ScoutActuatorState__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__init(scout_msgs__msg__ScoutActuatorState * msg);

/// Finalize msg/ScoutActuatorState message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutActuatorState__fini(scout_msgs__msg__ScoutActuatorState * msg);

/// Create msg/ScoutActuatorState message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * scout_msgs__msg__ScoutActuatorState__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutActuatorState *
scout_msgs__msg__ScoutActuatorState__create();

/// Destroy msg/ScoutActuatorState message.
/**
 * It calls
 * scout_msgs__msg__ScoutActuatorState__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutActuatorState__destroy(scout_msgs__msg__ScoutActuatorState * msg);

/// Check for msg/ScoutActuatorState message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__are_equal(const scout_msgs__msg__ScoutActuatorState * lhs, const scout_msgs__msg__ScoutActuatorState * rhs);

/// Copy a msg/ScoutActuatorState message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__copy(
  const scout_msgs__msg__ScoutActuatorState * input,
  scout_msgs__msg__ScoutActuatorState * output);

/// Initialize array of msg/ScoutActuatorState messages.
/**
 * It allocates the memory for the number of elements and calls
 * scout_msgs__msg__ScoutActuatorState__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__Sequence__init(scout_msgs__msg__ScoutActuatorState__Sequence * array, size_t size);

/// Finalize array of msg/ScoutActuatorState messages.
/**
 * It calls
 * scout_msgs__msg__ScoutActuatorState__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutActuatorState__Sequence__fini(scout_msgs__msg__ScoutActuatorState__Sequence * array);

/// Create array of msg/ScoutActuatorState messages.
/**
 * It allocates the memory for the array and calls
 * scout_msgs__msg__ScoutActuatorState__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutActuatorState__Sequence *
scout_msgs__msg__ScoutActuatorState__Sequence__create(size_t size);

/// Destroy array of msg/ScoutActuatorState messages.
/**
 * It calls
 * scout_msgs__msg__ScoutActuatorState__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutActuatorState__Sequence__destroy(scout_msgs__msg__ScoutActuatorState__Sequence * array);

/// Check for msg/ScoutActuatorState message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__Sequence__are_equal(const scout_msgs__msg__ScoutActuatorState__Sequence * lhs, const scout_msgs__msg__ScoutActuatorState__Sequence * rhs);

/// Copy an array of msg/ScoutActuatorState messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutActuatorState__Sequence__copy(
  const scout_msgs__msg__ScoutActuatorState__Sequence * input,
  scout_msgs__msg__ScoutActuatorState__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_ACTUATOR_STATE__FUNCTIONS_H_
