// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice

#ifndef SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__FUNCTIONS_H_
#define SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "scout_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "scout_msgs/msg/detail/scout_status__struct.h"

/// Initialize msg/ScoutStatus message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * scout_msgs__msg__ScoutStatus
 * )) before or use
 * scout_msgs__msg__ScoutStatus__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutStatus__init(scout_msgs__msg__ScoutStatus * msg);

/// Finalize msg/ScoutStatus message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutStatus__fini(scout_msgs__msg__ScoutStatus * msg);

/// Create msg/ScoutStatus message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * scout_msgs__msg__ScoutStatus__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutStatus *
scout_msgs__msg__ScoutStatus__create();

/// Destroy msg/ScoutStatus message.
/**
 * It calls
 * scout_msgs__msg__ScoutStatus__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutStatus__destroy(scout_msgs__msg__ScoutStatus * msg);

/// Check for msg/ScoutStatus message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutStatus__are_equal(const scout_msgs__msg__ScoutStatus * lhs, const scout_msgs__msg__ScoutStatus * rhs);

/// Copy a msg/ScoutStatus message.
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
scout_msgs__msg__ScoutStatus__copy(
  const scout_msgs__msg__ScoutStatus * input,
  scout_msgs__msg__ScoutStatus * output);

/// Initialize array of msg/ScoutStatus messages.
/**
 * It allocates the memory for the number of elements and calls
 * scout_msgs__msg__ScoutStatus__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutStatus__Sequence__init(scout_msgs__msg__ScoutStatus__Sequence * array, size_t size);

/// Finalize array of msg/ScoutStatus messages.
/**
 * It calls
 * scout_msgs__msg__ScoutStatus__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutStatus__Sequence__fini(scout_msgs__msg__ScoutStatus__Sequence * array);

/// Create array of msg/ScoutStatus messages.
/**
 * It allocates the memory for the array and calls
 * scout_msgs__msg__ScoutStatus__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
scout_msgs__msg__ScoutStatus__Sequence *
scout_msgs__msg__ScoutStatus__Sequence__create(size_t size);

/// Destroy array of msg/ScoutStatus messages.
/**
 * It calls
 * scout_msgs__msg__ScoutStatus__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
void
scout_msgs__msg__ScoutStatus__Sequence__destroy(scout_msgs__msg__ScoutStatus__Sequence * array);

/// Check for msg/ScoutStatus message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_scout_msgs
bool
scout_msgs__msg__ScoutStatus__Sequence__are_equal(const scout_msgs__msg__ScoutStatus__Sequence * lhs, const scout_msgs__msg__ScoutStatus__Sequence * rhs);

/// Copy an array of msg/ScoutStatus messages.
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
scout_msgs__msg__ScoutStatus__Sequence__copy(
  const scout_msgs__msg__ScoutStatus__Sequence * input,
  scout_msgs__msg__ScoutStatus__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // SCOUT_MSGS__MSG__DETAIL__SCOUT_STATUS__FUNCTIONS_H_
