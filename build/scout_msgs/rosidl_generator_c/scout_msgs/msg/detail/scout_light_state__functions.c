// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scout_msgs:msg/ScoutLightState.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_light_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
scout_msgs__msg__ScoutLightState__init(scout_msgs__msg__ScoutLightState * msg)
{
  if (!msg) {
    return false;
  }
  // mode
  // custom_value
  return true;
}

void
scout_msgs__msg__ScoutLightState__fini(scout_msgs__msg__ScoutLightState * msg)
{
  if (!msg) {
    return;
  }
  // mode
  // custom_value
}

bool
scout_msgs__msg__ScoutLightState__are_equal(const scout_msgs__msg__ScoutLightState * lhs, const scout_msgs__msg__ScoutLightState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // mode
  if (lhs->mode != rhs->mode) {
    return false;
  }
  // custom_value
  if (lhs->custom_value != rhs->custom_value) {
    return false;
  }
  return true;
}

bool
scout_msgs__msg__ScoutLightState__copy(
  const scout_msgs__msg__ScoutLightState * input,
  scout_msgs__msg__ScoutLightState * output)
{
  if (!input || !output) {
    return false;
  }
  // mode
  output->mode = input->mode;
  // custom_value
  output->custom_value = input->custom_value;
  return true;
}

scout_msgs__msg__ScoutLightState *
scout_msgs__msg__ScoutLightState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightState * msg = (scout_msgs__msg__ScoutLightState *)allocator.allocate(sizeof(scout_msgs__msg__ScoutLightState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scout_msgs__msg__ScoutLightState));
  bool success = scout_msgs__msg__ScoutLightState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scout_msgs__msg__ScoutLightState__destroy(scout_msgs__msg__ScoutLightState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scout_msgs__msg__ScoutLightState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scout_msgs__msg__ScoutLightState__Sequence__init(scout_msgs__msg__ScoutLightState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightState * data = NULL;

  if (size) {
    data = (scout_msgs__msg__ScoutLightState *)allocator.zero_allocate(size, sizeof(scout_msgs__msg__ScoutLightState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scout_msgs__msg__ScoutLightState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scout_msgs__msg__ScoutLightState__fini(&data[i - 1]);
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
scout_msgs__msg__ScoutLightState__Sequence__fini(scout_msgs__msg__ScoutLightState__Sequence * array)
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
      scout_msgs__msg__ScoutLightState__fini(&array->data[i]);
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

scout_msgs__msg__ScoutLightState__Sequence *
scout_msgs__msg__ScoutLightState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightState__Sequence * array = (scout_msgs__msg__ScoutLightState__Sequence *)allocator.allocate(sizeof(scout_msgs__msg__ScoutLightState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scout_msgs__msg__ScoutLightState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scout_msgs__msg__ScoutLightState__Sequence__destroy(scout_msgs__msg__ScoutLightState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scout_msgs__msg__ScoutLightState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scout_msgs__msg__ScoutLightState__Sequence__are_equal(const scout_msgs__msg__ScoutLightState__Sequence * lhs, const scout_msgs__msg__ScoutLightState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scout_msgs__msg__ScoutLightState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scout_msgs__msg__ScoutLightState__Sequence__copy(
  const scout_msgs__msg__ScoutLightState__Sequence * input,
  scout_msgs__msg__ScoutLightState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scout_msgs__msg__ScoutLightState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scout_msgs__msg__ScoutLightState * data =
      (scout_msgs__msg__ScoutLightState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scout_msgs__msg__ScoutLightState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scout_msgs__msg__ScoutLightState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scout_msgs__msg__ScoutLightState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
