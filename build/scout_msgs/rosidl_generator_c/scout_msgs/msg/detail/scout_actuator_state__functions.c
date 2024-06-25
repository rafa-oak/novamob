// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scout_msgs:msg/ScoutActuatorState.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_actuator_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
scout_msgs__msg__ScoutActuatorState__init(scout_msgs__msg__ScoutActuatorState * msg)
{
  if (!msg) {
    return false;
  }
  // motor_id
  // rpm
  // current
  // pulse_count
  // driver_voltage
  // driver_temperature
  // motor_temperature
  // driver_state
  return true;
}

void
scout_msgs__msg__ScoutActuatorState__fini(scout_msgs__msg__ScoutActuatorState * msg)
{
  if (!msg) {
    return;
  }
  // motor_id
  // rpm
  // current
  // pulse_count
  // driver_voltage
  // driver_temperature
  // motor_temperature
  // driver_state
}

bool
scout_msgs__msg__ScoutActuatorState__are_equal(const scout_msgs__msg__ScoutActuatorState * lhs, const scout_msgs__msg__ScoutActuatorState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // motor_id
  if (lhs->motor_id != rhs->motor_id) {
    return false;
  }
  // rpm
  if (lhs->rpm != rhs->rpm) {
    return false;
  }
  // current
  if (lhs->current != rhs->current) {
    return false;
  }
  // pulse_count
  if (lhs->pulse_count != rhs->pulse_count) {
    return false;
  }
  // driver_voltage
  if (lhs->driver_voltage != rhs->driver_voltage) {
    return false;
  }
  // driver_temperature
  if (lhs->driver_temperature != rhs->driver_temperature) {
    return false;
  }
  // motor_temperature
  if (lhs->motor_temperature != rhs->motor_temperature) {
    return false;
  }
  // driver_state
  if (lhs->driver_state != rhs->driver_state) {
    return false;
  }
  return true;
}

bool
scout_msgs__msg__ScoutActuatorState__copy(
  const scout_msgs__msg__ScoutActuatorState * input,
  scout_msgs__msg__ScoutActuatorState * output)
{
  if (!input || !output) {
    return false;
  }
  // motor_id
  output->motor_id = input->motor_id;
  // rpm
  output->rpm = input->rpm;
  // current
  output->current = input->current;
  // pulse_count
  output->pulse_count = input->pulse_count;
  // driver_voltage
  output->driver_voltage = input->driver_voltage;
  // driver_temperature
  output->driver_temperature = input->driver_temperature;
  // motor_temperature
  output->motor_temperature = input->motor_temperature;
  // driver_state
  output->driver_state = input->driver_state;
  return true;
}

scout_msgs__msg__ScoutActuatorState *
scout_msgs__msg__ScoutActuatorState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutActuatorState * msg = (scout_msgs__msg__ScoutActuatorState *)allocator.allocate(sizeof(scout_msgs__msg__ScoutActuatorState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scout_msgs__msg__ScoutActuatorState));
  bool success = scout_msgs__msg__ScoutActuatorState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scout_msgs__msg__ScoutActuatorState__destroy(scout_msgs__msg__ScoutActuatorState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scout_msgs__msg__ScoutActuatorState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scout_msgs__msg__ScoutActuatorState__Sequence__init(scout_msgs__msg__ScoutActuatorState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutActuatorState * data = NULL;

  if (size) {
    data = (scout_msgs__msg__ScoutActuatorState *)allocator.zero_allocate(size, sizeof(scout_msgs__msg__ScoutActuatorState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scout_msgs__msg__ScoutActuatorState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scout_msgs__msg__ScoutActuatorState__fini(&data[i - 1]);
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
scout_msgs__msg__ScoutActuatorState__Sequence__fini(scout_msgs__msg__ScoutActuatorState__Sequence * array)
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
      scout_msgs__msg__ScoutActuatorState__fini(&array->data[i]);
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

scout_msgs__msg__ScoutActuatorState__Sequence *
scout_msgs__msg__ScoutActuatorState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutActuatorState__Sequence * array = (scout_msgs__msg__ScoutActuatorState__Sequence *)allocator.allocate(sizeof(scout_msgs__msg__ScoutActuatorState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scout_msgs__msg__ScoutActuatorState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scout_msgs__msg__ScoutActuatorState__Sequence__destroy(scout_msgs__msg__ScoutActuatorState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scout_msgs__msg__ScoutActuatorState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scout_msgs__msg__ScoutActuatorState__Sequence__are_equal(const scout_msgs__msg__ScoutActuatorState__Sequence * lhs, const scout_msgs__msg__ScoutActuatorState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scout_msgs__msg__ScoutActuatorState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scout_msgs__msg__ScoutActuatorState__Sequence__copy(
  const scout_msgs__msg__ScoutActuatorState__Sequence * input,
  scout_msgs__msg__ScoutActuatorState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scout_msgs__msg__ScoutActuatorState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scout_msgs__msg__ScoutActuatorState * data =
      (scout_msgs__msg__ScoutActuatorState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scout_msgs__msg__ScoutActuatorState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scout_msgs__msg__ScoutActuatorState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scout_msgs__msg__ScoutActuatorState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
