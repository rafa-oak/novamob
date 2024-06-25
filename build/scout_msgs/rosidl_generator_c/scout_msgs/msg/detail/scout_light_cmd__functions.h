// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__FUNCTIONS_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "scout_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "scout_msgs/msg/detail/scout_light_cmd__struct.h"

/// Initialize msg/ScoutLightCmd message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * scout_msgs__msg__ScoutLightCmd
 * )) before or use
 * scout_msgs__msg__ScoutLightCmd__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutLightCmd__init(scout_msgs__msg__ScoutLightCmd * msg);

/// Finalize msg/ScoutLightCmd message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutLightCmd__fini(scout_msgs__msg__ScoutLightCmd * msg);

/// Create msg/ScoutLightCmd message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * scout_msgs__msg__ScoutLightCmd__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutLightCmd *
scout_msgs__msg__ScoutLightCmd__create();

/// Destroy msg/ScoutLightCmd message.
/**
 * It calls
 * scout_msgs__msg__ScoutLightCmd__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutLightCmd__destroy(scout_msgs__msg__ScoutLightCmd * msg);

/// Check for msg/ScoutLightCmd message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutLightCmd__are_equal(const scout_msgs__msg__ScoutLightCmd * lhs, const scout_msgs__msg__ScoutLightCmd * rhs);

/// Copy a msg/ScoutLightCmd message.
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
scout_msgs__msg__ScoutLightCmd__copy(
  const scout_msgs__msg__ScoutLightCmd * input,
  scout_msgs__msg__ScoutLightCmd * output);

/// Initialize array of msg/ScoutLightCmd messages.
/**
 * It allocates the memory for the number of elements and calls
 * scout_msgs__msg__ScoutLightCmd__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutLightCmd__Sequence__init(scout_msgs__msg__ScoutLightCmd__Sequence * array, size_t size);

/// Finalize array of msg/ScoutLightCmd messages.
/**
 * It calls
 * scout_msgs__msg__ScoutLightCmd__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutLightCmd__Sequence__fini(scout_msgs__msg__ScoutLightCmd__Sequence * array);

/// Create array of msg/ScoutLightCmd messages.
/**
 * It allocates the memory for the array and calls
 * scout_msgs__msg__ScoutLightCmd__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutLightCmd__Sequence *
scout_msgs__msg__ScoutLightCmd__Sequence__create(size_t size);

/// Destroy array of msg/ScoutLightCmd messages.
/**
 * It calls
 * scout_msgs__msg__ScoutLightCmd__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutLightCmd__Sequence__destroy(scout_msgs__msg__ScoutLightCmd__Sequence * array);

/// Check for msg/ScoutLightCmd message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutLightCmd__Sequence__are_equal(const scout_msgs__msg__ScoutLightCmd__Sequence * lhs, const scout_msgs__msg__ScoutLightCmd__Sequence * rhs);

/// Copy an array of msg/ScoutLightCmd messages.
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
scout_msgs__msg__ScoutLightCmd__Sequence__copy(
  const scout_msgs__msg__ScoutLightCmd__Sequence * input,
  scout_msgs__msg__ScoutLightCmd__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_LIGHT_CMD__FUNCTIONS_H_
