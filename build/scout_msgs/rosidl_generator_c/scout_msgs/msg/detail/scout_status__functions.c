// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scout_msgs:msg/ScoutStatus.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_status__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"
// Member `actuator_states`
#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"
// Member `front_light_state`
// Member `rear_light_state`
#include "scout_msgs/msg/detail/scout_light_state__functions.h"

bool
scout_msgs__msg__ScoutStatus__init(scout_msgs__msg__ScoutStatus * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    scout_msgs__msg__ScoutStatus__fini(msg);
    return false;
  }
  // linear_velocity
  // angular_velocity
  // vehicle_state
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 4; ++i) {
    if (!scout_msgs__msg__ScoutActuatorState__init(&msg->actuator_states[i])) {
      scout_msgs__msg__ScoutStatus__fini(msg);
      return false;
    }
  }
  // light_control_enabled
  // front_light_state
  if (!scout_msgs__msg__ScoutLightState__init(&msg->front_light_state)) {
    scout_msgs__msg__ScoutStatus__fini(msg);
    return false;
  }
  // rear_light_state
  if (!scout_msgs__msg__ScoutLightState__init(&msg->rear_light_state)) {
    scout_msgs__msg__ScoutStatus__fini(msg);
    return false;
  }
  return true;
}

void
scout_msgs__msg__ScoutStatus__fini(scout_msgs__msg__ScoutStatus * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // linear_velocity
  // angular_velocity
  // vehicle_state
  // control_mode
  // error_code
  // battery_voltage
  // actuator_states
  for (size_t i = 0; i < 4; ++i) {
    scout_msgs__msg__ScoutActuatorState__fini(&msg->actuator_states[i]);
  }
  // light_control_enabled
  // front_light_state
  scout_msgs__msg__ScoutLightState__fini(&msg->front_light_state);
  // rear_light_state
  scout_msgs__msg__ScoutLightState__fini(&msg->rear_light_state);
}

bool
scout_msgs__msg__ScoutStatus__are_equal(const scout_msgs__msg__ScoutStatus * lhs, const scout_msgs__msg__ScoutStatus * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__are_equal(
      &(lhs->header), &(rhs->header)))
  {
    return false;
  }
  // linear_velocity
  if (lhs->linear_velocity != rhs->linear_velocity) {
    return false;
  }
  // angular_velocity
  if (lhs->angular_velocity != rhs->angular_velocity) {
    return false;
  }
  // vehicle_state
  if (lhs->vehicle_state != rhs->vehicle_state) {
    return false;
  }
  // control_mode
  if (lhs->control_mode != rhs->control_mode) {
    return false;
  }
  // error_code
  if (lhs->error_code != rhs->error_code) {
    return false;
  }
  // battery_voltage
  if (lhs->battery_voltage != rhs->battery_voltage) {
    return false;
  }
  // actuator_states
  for (size_t i = 0; i < 4; ++i) {
    if (!scout_msgs__msg__ScoutActuatorState__are_equal(
        &(lhs->actuator_states[i]), &(rhs->actuator_states[i])))
    {
      return false;
    }
  }
  // light_control_enabled
  if (lhs->light_control_enabled != rhs->light_control_enabled) {
    return false;
  }
  // front_light_state
  if (!scout_msgs__msg__ScoutLightState__are_equal(
      &(lhs->front_light_state), &(rhs->front_light_state)))
  {
    return false;
  }
  // rear_light_state
  if (!scout_msgs__msg__ScoutLightState__are_equal(
      &(lhs->rear_light_state), &(rhs->rear_light_state)))
  {
    return false;
  }
  return true;
}

bool
scout_msgs__msg__ScoutStatus__copy(
  const scout_msgs__msg__ScoutStatus * input,
  scout_msgs__msg__ScoutStatus * output)
{
  if (!input || !output) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__copy(
      &(input->header), &(output->header)))
  {
    return false;
  }
  // linear_velocity
  output->linear_velocity = input->linear_velocity;
  // angular_velocity
  output->angular_velocity = input->angular_velocity;
  // vehicle_state
  output->vehicle_state = input->vehicle_state;
  // control_mode
  output->control_mode = input->control_mode;
  // error_code
  output->error_code = input->error_code;
  // battery_voltage
  output->battery_voltage = input->battery_voltage;
  // actuator_states
  for (size_t i = 0; i < 4; ++i) {
    if (!scout_msgs__msg__ScoutActuatorState__copy(
        &(input->actuator_states[i]), &(output->actuator_states[i])))
    {
      return false;
    }
  }
  // light_control_enabled
  output->light_control_enabled = input->light_control_enabled;
  // front_light_state
  if (!scout_msgs__msg__ScoutLightState__copy(
      &(input->front_light_state), &(output->front_light_state)))
  {
    return false;
  }
  // rear_light_state
  if (!scout_msgs__msg__ScoutLightState__copy(
      &(input->rear_light_state), &(output->rear_light_state)))
  {
    return false;
  }
  return true;
}

scout_msgs__msg__ScoutStatus *
scout_msgs__msg__ScoutStatus__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutStatus * msg = (scout_msgs__msg__ScoutStatus *)allocator.allocate(sizeof(scout_msgs__msg__ScoutStatus), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scout_msgs__msg__ScoutStatus));
  bool success = scout_msgs__msg__ScoutStatus__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scout_msgs__msg__ScoutStatus__destroy(scout_msgs__msg__ScoutStatus * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scout_msgs__msg__ScoutStatus__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scout_msgs__msg__ScoutStatus__Sequence__init(scout_msgs__msg__ScoutStatus__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutStatus * data = NULL;

  if (size) {
    data = (scout_msgs__msg__ScoutStatus *)allocator.zero_allocate(size, sizeof(scout_msgs__msg__ScoutStatus), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scout_msgs__msg__ScoutStatus__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scout_msgs__msg__ScoutStatus__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
scout_msgs__msg__ScoutStatus__Sequence__fini(scout_msgs__msg__ScoutStatus__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      scout_msgs__msg__ScoutStatus__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

scout_msgs__msg__ScoutStatus__Sequence *
scout_msgs__msg__ScoutStatus__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutStatus__Sequence * array = (scout_msgs__msg__ScoutStatus__Sequence *)allocator.allocate(sizeof(scout_msgs__msg__ScoutStatus__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scout_msgs__msg__ScoutStatus__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scout_msgs__msg__ScoutStatus__Sequence__destroy(scout_msgs__msg__ScoutStatus__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scout_msgs__msg__ScoutStatus__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scout_msgs__msg__ScoutStatus__Sequence__are_equal(const scout_msgs__msg__ScoutStatus__Sequence * lhs, const scout_msgs__msg__ScoutStatus__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scout_msgs__msg__ScoutStatus__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scout_msgs__msg__ScoutStatus__Sequence__copy(
  const scout_msgs__msg__ScoutStatus__Sequence * input,
  scout_msgs__msg__ScoutStatus__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scout_msgs__msg__ScoutStatus);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scout_msgs__msg__ScoutStatus * data =
      (scout_msgs__msg__ScoutStatus *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scout_msgs__msg__ScoutStatus__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scout_msgs__msg__ScoutStatus__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scout_msgs__msg__ScoutStatus__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
