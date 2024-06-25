// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_status__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "scout_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "scout_msgs/msg/detail/scout_status__struct.h"
#include "scout_msgs/msg/detail/scout_status__functions.h"
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

#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"  // actuator_states
#include "scout_msgs/msg/detail/scout_light_state__functions.h"  // front_light_state, rear_light_state
#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
size_t get_serialized_size_scout_msgs__msg__ScoutActuatorState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_scout_msgs__msg__ScoutActuatorState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutActuatorState)();
size_t get_serialized_size_scout_msgs__msg__ScoutLightState(
  const void * untyped_ros_message,
  size_t current_alignment);

size_t max_serialized_size_scout_msgs__msg__ScoutLightState(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightState)();
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_scout_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _ScoutStatus__ros_msg_type = scout_msgs__msg__ScoutStatus;

static bool _ScoutStatus__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _ScoutStatus__ros_msg_type * ros_message = static_cast<const _ScoutStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: linear_velocity
  {
    cdr << ros_message->linear_velocity;
  }

  // Field name: angular_velocity
  {
    cdr << ros_message->angular_velocity;
  }

  // Field name: vehicle_state
  {
    cdr << ros_message->vehicle_state;
  }

  // Field name: control_mode
  {
    cdr << ros_message->control_mode;
  }

  // Field name: error_code
  {
    cdr << ros_message->error_code;
  }

  // Field name: battery_voltage
  {
    cdr << ros_message->battery_voltage;
  }

  // Field name: actuator_states
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutActuatorState
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->actuator_states;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_serialize(
          &array_ptr[i], cdr))
      {
        return false;
      }
    }
  }

  // Field name: light_control_enabled
  {
    cdr << (ros_message->light_control_enabled ? true : false);
  }

  // Field name: front_light_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->front_light_state, cdr))
    {
      return false;
    }
  }

  // Field name: rear_light_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightState
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->rear_light_state, cdr))
    {
      return false;
    }
  }

  return true;
}

static bool _ScoutStatus__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _ScoutStatus__ros_msg_type * ros_message = static_cast<_ScoutStatus__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: linear_velocity
  {
    cdr >> ros_message->linear_velocity;
  }

  // Field name: angular_velocity
  {
    cdr >> ros_message->angular_velocity;
  }

  // Field name: vehicle_state
  {
    cdr >> ros_message->vehicle_state;
  }

  // Field name: control_mode
  {
    cdr >> ros_message->control_mode;
  }

  // Field name: error_code
  {
    cdr >> ros_message->error_code;
  }

  // Field name: battery_voltage
  {
    cdr >> ros_message->battery_voltage;
  }

  // Field name: actuator_states
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutActuatorState
      )()->data);
    size_t size = 4;
    auto array_ptr = ros_message->actuator_states;
    for (size_t i = 0; i < size; ++i) {
      if (!callbacks->cdr_deserialize(
          cdr, &array_ptr[i]))
      {
        return false;
      }
    }
  }

  // Field name: light_control_enabled
  {
    uint8_t tmp;
    cdr >> tmp;
    ros_message->light_control_enabled = tmp ? true : false;
  }

  // Field name: front_light_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->front_light_state))
    {
      return false;
    }
  }

  // Field name: rear_light_state
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutLightState
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->rear_light_state))
    {
      return false;
    }
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t get_serialized_size_scout_msgs__msg__ScoutStatus(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _ScoutStatus__ros_msg_type * ros_message = static_cast<const _ScoutStatus__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name linear_velocity
  {
    size_t item_size = sizeof(ros_message->linear_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name angular_velocity
  {
    size_t item_size = sizeof(ros_message->angular_velocity);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vehicle_state
  {
    size_t item_size = sizeof(ros_message->vehicle_state);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name control_mode
  {
    size_t item_size = sizeof(ros_message->control_mode);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name error_code
  {
    size_t item_size = sizeof(ros_message->error_code);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name battery_voltage
  {
    size_t item_size = sizeof(ros_message->battery_voltage);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name actuator_states
  {
    size_t array_size = 4;
    auto array_ptr = ros_message->actuator_states;

    for (size_t index = 0; index < array_size; ++index) {
      current_alignment += get_serialized_size_scout_msgs__msg__ScoutActuatorState(
        &array_ptr[index], current_alignment);
    }
  }
  // field.name light_control_enabled
  {
    size_t item_size = sizeof(ros_message->light_control_enabled);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name front_light_state

  current_alignment += get_serialized_size_scout_msgs__msg__ScoutLightState(
    &(ros_message->front_light_state), current_alignment);
  // field.name rear_light_state

  current_alignment += get_serialized_size_scout_msgs__msg__ScoutLightState(
    &(ros_message->rear_light_state), current_alignment);

  return current_alignment - initial_alignment;
}

static uint32_t _ScoutStatus__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_scout_msgs__msg__ScoutStatus(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_scout_msgs
size_t max_serialized_size_scout_msgs__msg__ScoutStatus(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: linear_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: angular_velocity
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: vehicle_state
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: control_mode
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: error_code
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint16_t);
    current_alignment += array_size * sizeof(uint16_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint16_t));
  }
  // member: battery_voltage
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint64_t);
    current_alignment += array_size * sizeof(uint64_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint64_t));
  }
  // member: actuator_states
  {
    size_t array_size = 4;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_scout_msgs__msg__ScoutActuatorState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: light_control_enabled
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint8_t);
    current_alignment += array_size * sizeof(uint8_t);
  }
  // member: front_light_state
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_scout_msgs__msg__ScoutLightState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: rear_light_state
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_scout_msgs__msg__ScoutLightState(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = scout_msgs__msg__ScoutStatus;
    is_plain =
      (
      offsetof(DataType, rear_light_state) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _ScoutStatus__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_scout_msgs__msg__ScoutStatus(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_ScoutStatus = {
  "scout_msgs::msg",
  "ScoutStatus",
  _ScoutStatus__cdr_serialize,
  _ScoutStatus__cdr_deserialize,
  _ScoutStatus__get_serialized_size,
  _ScoutStatus__max_serialized_size
};

static rosidl_message_type_support_t _ScoutStatus__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_ScoutStatus,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, scout_msgs, msg, ScoutStatus)() {
  return &_ScoutStatus__type_support;
}

#if defined(__cplusplus)
}
#endif
