// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "scout_msgs/msg/detail/scout_actuator_state__rosidl_typesupport_introspection_c.h"
#include "scout_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"
#include "scout_msgs/msg/detail/scout_actuator_state__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  scout_msgs__msg__ScoutActuatorState__init(message_memory);
}

void scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_fini_function(void * message_memory)
{
  scout_msgs__msg__ScoutActuatorState__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_member_array[8] = {
  {
    "motor_id",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, motor_id),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "rpm",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT16,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, rpm),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "current",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_DOUBLE,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, current),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "pulse_count",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, pulse_count),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_voltage",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, driver_voltage),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_FLOAT,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, driver_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "motor_temperature",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, motor_temperature),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "driver_state",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_UINT8,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(scout_msgs__msg__ScoutActuatorState, driver_state),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_members = {
  "scout_msgs__msg",  // message namespace
  "ScoutActuatorState",  // message name
  8,  // number of fields
  sizeof(scout_msgs__msg__ScoutActuatorState),
  scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_member_array,  // message members
  scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_init_function,  // function to initialize message memory (memory has to be allocated)
  scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_type_support_handle = {
  0,
  &scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_scout_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, scout_msgs, msg, ScoutActuatorState)() {
  if (!scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_type_support_handle.typesupport_identifier) {
    scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &scout_msgs__msg__ScoutActuatorState__rosidl_typesupport_introspection_c__ScoutActuatorState_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
