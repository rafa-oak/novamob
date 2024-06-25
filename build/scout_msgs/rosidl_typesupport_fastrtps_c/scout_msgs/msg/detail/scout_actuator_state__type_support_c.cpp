// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_actuator_state__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scout_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scout_msgs/msg/detail/scout_actuator_state__struct.h"
#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"
#include "fastcdr/Cdr.h"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif


// forward declare type support functions


using _ScoutActuatorState__ros_msg_type = scout_msgs__msg__ScoutActuatorState;

static bool _ScoutActuatorState__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ScoutActuatorState__ros_msg_type * ros_message = static_cast<const _ScoutActuatorState__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_id
  {
    cdr << ros_message->motor_id;
  }

  // Field name: rpm
  {
    cdr << ros_message->rpm;
  }

  // Field name: current
  {
    cdr << ros_message->current;
  }

  // Field name: pulse_count
  {
    cdr << ros_message->pulse_count;
  }

  // Field name: driver_voltage
  {
    cdr << ros_message->driver_voltage;
  }

  // Field name: driver_temperature
  {
    cdr << ros_message->driver_temperature;
  }

  // Field name: motor_temperature
  {
    cdr << ros_message->motor_temperature;
  }

  // Field name: driver_state
  {
    cdr << ros_message->driver_state;
  }

  return true;
}

static bool _ScoutActuatorState__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ScoutActuatorState__ros_msg_type * ros_message = static_cast<_ScoutActuatorState__ros_msg_type *>(untyped_ros_message);
  // Field name: motor_id
  {
    cdr >> ros_message->motor_id;
  }

  // Field name: rpm
  {
    cdr >> ros_message->rpm;
  }

  // Field name: current
  {
    cdr >> ros_message->current;
  }

  // Field name: pulse_count
  {
    cdr >> ros_message->pulse_count;
  }

  // Field name: driver_voltage
  {
    cdr >> ros_message->driver_voltage;
  }

  // Field name: driver_temperature
  {
    cdr >> ros_message->driver_temperature;
  }

  // Field name: motor_temperature
  {
    cdr >> ros_message->motor_temperature;
  }

  // Field name: driver_state
  {
    cdr >> ros_message->driver_state;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t get_serialized_size_scout_msgs__msg__ScoutActuatorState(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScoutActuatorState__ros_msg_type * ros_message = static_cast<const _ScoutActuatorState__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name motor_id
  {
    size_t item_size = sizeof(ros_message->motor_id);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name rpm
  {
    size_t item_size = sizeof(ros_message->rpm);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name current
  {
    size_t item_size = sizeof(ros_message->current);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name pulse_count
  {
    size_t item_size = sizeof(ros_message->pulse_count);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_voltage
  {
    size_t item_size = sizeof(ros_message->driver_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_temperature
  {
    size_t item_size = sizeof(ros_message->driver_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name motor_temperature
  {
    size_t item_size = sizeof(ros_message->motor_temperature);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name driver_state
  {
    size_t item_size = sizeof(ros_message->driver_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _ScoutActuatorState__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scout_msgs__msg__ScoutActuatorState(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t max_serialized_size_scout_msgs__msg__ScoutActuatorState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;

  // member: motor_id
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: rpm
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: current
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: pulse_count
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: driver_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: driver_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: motor_temperature
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: driver_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = scout_msgs__msg__ScoutActuatorState;
    is_plain =
      (
      offsetof(DataType, driver_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ScoutActuatorState__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_scout_msgs__msg__ScoutActuatorState(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ScoutActuatorState = {
  "scout_msgs::msg",
  "ScoutActuatorState",
  _ScoutActuatorState__cdr_serialize,
  _ScoutActuatorState__cdr_deserialize,
  _ScoutActuatorState__get_serialized_size,
  _ScoutActuatorState__max_serialized_size
};

static rosidl_message_type_support_t _ScoutActuatorState__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ScoutActuatorState,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutActuatorState)() {
  return &_ScoutActuatorState__type_support;
}

#if defined(__cplusplus)
}
#endif
