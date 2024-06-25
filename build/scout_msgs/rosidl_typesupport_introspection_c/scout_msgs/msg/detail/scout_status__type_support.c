// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scout_msgs/msg/detail/scout_status__rosidl_typesupport_introspection_c.h"
#include "scout_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scout_msgs/msg/detail/scout_status__functions.h"
#include "scout_msgs/msg/detail/scout_status__struct.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/header.h"
// Member `header`
#include "std_msgs/msg/detail/header__rosidl_typesupport_introspection_c.h"
// Member `actuator_states`
#include "scout_msgs/msg/scout_actuator_state.h"
// Member `actuator_states`
#include "scout_msgs/msg/detail/scout_actuator_state__rosidl_typesupport_introspection_c.h"
// Member `front_light_state`
// Member `rear_light_state`
#include "scout_msgs/msg/scout_light_state.h"
// Member `front_light_state`
// Member `rear_light_state`
#include "scout_msgs/msg/detail/scout_light_state__rosidl_typesupport_introspection_c.h"

#ifdef __cplusplus
extern "C"
{
#endif

void scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scout_msgs__msg__ScoutStatus__init(message_memory);
}

void scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_fini_function(void * message_memory)
{
  scout_msgs__msg__ScoutStatus__fini(message_memory);
}

size_t scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__size_function__ScoutStatus__actuator_states(
  const void * untyped_member)
{
  (void)untyped_member;
  return 4;
}

const void * scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_const_function__ScoutStatus__actuator_states(
  const void * untyped_member, size_t index)
{
  const scout_msgs__msg__ScoutActuatorState * member =
    (const scout_msgs__msg__ScoutActuatorState *)(untyped_member);
  return &member[index];
}

void * scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_function__ScoutStatus__actuator_states(
  void * untyped_member, size_t index)
{
  scout_msgs__msg__ScoutActuatorState * member =
    (scout_msgs__msg__ScoutActuatorState *)(untyped_member);
  return &member[index];
}

void scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__fetch_function__ScoutStatus__actuator_states(
  const void * untyped_member, size_t index, void * untyped_value)
{
  const scout_msgs__msg__ScoutActuatorState * item =
    ((const scout_msgs__msg__ScoutActuatorState *)
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_const_function__ScoutStatus__actuator_states(untyped_member, index));
  scout_msgs__msg__ScoutActuatorState * value =
    (scout_msgs__msg__ScoutActuatorState *)(untyped_value);
  *value = *item;
}

void scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__assign_function__ScoutStatus__actuator_states(
  void * untyped_member, size_t index, const void * untyped_value)
{
  scout_msgs__msg__ScoutActuatorState * item =
    ((scout_msgs__msg__ScoutActuatorState *)
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_function__ScoutStatus__actuator_states(untyped_member, index));
  const scout_msgs__msg__ScoutActuatorState * value =
    (const scout_msgs__msg__ScoutActuatorState *)(untyped_value);
  *item = *value;
}

static rosidl_typesupport_introspection_c__MessageMember scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array[11] = {
  {
    "header",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, header),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "linear_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, linear_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "angular_velocity",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, angular_velocity),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "vehicle_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, vehicle_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "control_mode",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, control_mode),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "error_code",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, error_code),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "battery_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, battery_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "actuator_states",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    true,  // is array
    4,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, actuator_states),  // bytes offset in struct
    NULL,  // default value
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__size_function__ScoutStatus__actuator_states,  // size() function pointer
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_const_function__ScoutStatus__actuator_states,  // get_const(index) function pointer
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__get_function__ScoutStatus__actuator_states,  // get(index) function pointer
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__fetch_function__ScoutStatus__actuator_states,  // fetch(index, &value) function pointer
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__assign_function__ScoutStatus__actuator_states,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "light_control_enabled",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_BOOLEAN,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, light_control_enabled),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "front_light_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, front_light_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rear_light_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_MESSAGE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message (initialized later)
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutStatus, rear_light_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_members = {
  "scout_msgs__msg",  // message namespace
  "ScoutStatus",  // message name
  11,  // number of fields
  sizeof(scout_msgs__msg__ScoutStatus),
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array,  // message members
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_init_function,  // function to initialize message memory (memory has to be allocated)
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_type_support_handle = {
  0,
  &scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scout_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scout_msgs, msg, ScoutStatus)() {
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array[0].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, std_msgs, msg, Header)();
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array[7].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scout_msgs, msg, ScoutActuatorState)();
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array[9].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scout_msgs, msg, ScoutLightState)();
  scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_member_array[10].members_ =
    ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scout_msgs, msg, ScoutLightState)();
  if (!scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_type_support_handle.typesupport_identifier) {
    scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &scout_msgs__msg__ScoutStatus__rosidl_typesupport_introspection_c__ScoutStatus_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
