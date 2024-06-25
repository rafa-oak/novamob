// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scout_msgs:msg/ScoutLightCmd.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_light_cmd__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
scout_msgs__msg__ScoutLightCmd__init(scout_msgs__msg__ScoutLightCmd * msg)
{
  if (!msg) {
    return false;
  }
  // cmd_ctrl_allowed
  // front_mode
  // front_custom_value
  // rear_mode
  // rear_custom_value
  return true;
}

void
scout_msgs__msg__ScoutLightCmd__fini(scout_msgs__msg__ScoutLightCmd * msg)
{
  if (!msg) {
    return;
  }
  // cmd_ctrl_allowed
  // front_mode
  // front_custom_value
  // rear_mode
  // rear_custom_value
}

bool
scout_msgs__msg__ScoutLightCmd__are_equal(const scout_msgs__msg__ScoutLightCmd * lhs, const scout_msgs__msg__ScoutLightCmd * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // cmd_ctrl_allowed
  if (lhs->cmd_ctrl_allowed != rhs->cmd_ctrl_allowed) {
    return false;
  }
  // front_mode
  if (lhs->front_mode != rhs->front_mode) {
    return false;
  }
  // front_custom_value
  if (lhs->front_custom_value != rhs->front_custom_value) {
    return false;
  }
  // rear_mode
  if (lhs->rear_mode != rhs->rear_mode) {
    return false;
  }
  // rear_custom_value
  if (lhs->rear_custom_value != rhs->rear_custom_value) {
    return false;
  }
  return true;
}

bool
scout_msgs__msg__ScoutLightCmd__copy(
  const scout_msgs__msg__ScoutLightCmd * input,
  scout_msgs__msg__ScoutLightCmd * output)
{
  if (!input || !output) {
    return false;
  }
  // cmd_ctrl_allowed
  output->cmd_ctrl_allowed = input->cmd_ctrl_allowed;
  // front_mode
  output->front_mode = input->front_mode;
  // front_custom_value
  output->front_custom_value = input->front_custom_value;
  // rear_mode
  output->rear_mode = input->rear_mode;
  // rear_custom_value
  output->rear_custom_value = input->rear_custom_value;
  return true;
}

scout_msgs__msg__ScoutLightCmd *
scout_msgs__msg__ScoutLightCmd__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightCmd * msg = (scout_msgs__msg__ScoutLightCmd *)allocator.allocate(sizeof(scout_msgs__msg__ScoutLightCmd), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scout_msgs__msg__ScoutLightCmd));
  bool success = scout_msgs__msg__ScoutLightCmd__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scout_msgs__msg__ScoutLightCmd__destroy(scout_msgs__msg__ScoutLightCmd * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scout_msgs__msg__ScoutLightCmd__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scout_msgs__msg__ScoutLightCmd__Sequence__init(scout_msgs__msg__ScoutLightCmd__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightCmd * data = NULL;

  if (size) {
    data = (scout_msgs__msg__ScoutLightCmd *)allocator.zero_allocate(size, sizeof(scout_msgs__msg__ScoutLightCmd), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scout_msgs__msg__ScoutLightCmd__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scout_msgs__msg__ScoutLightCmd__fini(&data[i - 1]);
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
scout_msgs__msg__ScoutLightCmd__Sequence__fini(scout_msgs__msg__ScoutLightCmd__Sequence * array)
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
      scout_msgs__msg__ScoutLightCmd__fini(&array->data[i]);
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

scout_msgs__msg__ScoutLightCmd__Sequence *
scout_msgs__msg__ScoutLightCmd__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutLightCmd__Sequence * array = (scout_msgs__msg__ScoutLightCmd__Sequence *)allocator.allocate(sizeof(scout_msgs__msg__ScoutLightCmd__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scout_msgs__msg__ScoutLightCmd__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scout_msgs__msg__ScoutLightCmd__Sequence__destroy(scout_msgs__msg__ScoutLightCmd__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scout_msgs__msg__ScoutLightCmd__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scout_msgs__msg__ScoutLightCmd__Sequence__are_equal(const scout_msgs__msg__ScoutLightCmd__Sequence * lhs, const scout_msgs__msg__ScoutLightCmd__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scout_msgs__msg__ScoutLightCmd__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scout_msgs__msg__ScoutLightCmd__Sequence__copy(
  const scout_msgs__msg__ScoutLightCmd__Sequence * input,
  scout_msgs__msg__ScoutLightCmd__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scout_msgs__msg__ScoutLightCmd);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scout_msgs__msg__ScoutLightCmd * data =
      (scout_msgs__msg__ScoutLightCmd *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scout_msgs__msg__ScoutLightCmd__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scout_msgs__msg__ScoutLightCmd__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scout_msgs__msg__ScoutLightCmd__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
