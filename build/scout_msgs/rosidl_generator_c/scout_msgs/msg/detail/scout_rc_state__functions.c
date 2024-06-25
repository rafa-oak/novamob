// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from scout_msgs:msg/ScoutRCState.idl
// generated code does not contain a copyright notice
#include "scout_msgs/msg/detail/scout_rc_state__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
scout_msgs__msg__ScoutRCState__init(scout_msgs__msg__ScoutRCState * msg)
{
  if (!msg) {
    return false;
  }
  // swa
  // swb
  // swc
  // swd
  // stick_right_v
  // stick_right_h
  // stick_left_v
  // stick_left_h
  // var_a
  return true;
}

void
scout_msgs__msg__ScoutRCState__fini(scout_msgs__msg__ScoutRCState * msg)
{
  if (!msg) {
    return;
  }
  // swa
  // swb
  // swc
  // swd
  // stick_right_v
  // stick_right_h
  // stick_left_v
  // stick_left_h
  // var_a
}

bool
scout_msgs__msg__ScoutRCState__are_equal(const scout_msgs__msg__ScoutRCState * lhs, const scout_msgs__msg__ScoutRCState * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // swa
  if (lhs->swa != rhs->swa) {
    return false;
  }
  // swb
  if (lhs->swb != rhs->swb) {
    return false;
  }
  // swc
  if (lhs->swc != rhs->swc) {
    return false;
  }
  // swd
  if (lhs->swd != rhs->swd) {
    return false;
  }
  // stick_right_v
  if (lhs->stick_right_v != rhs->stick_right_v) {
    return false;
  }
  // stick_right_h
  if (lhs->stick_right_h != rhs->stick_right_h) {
    return false;
  }
  // stick_left_v
  if (lhs->stick_left_v != rhs->stick_left_v) {
    return false;
  }
  // stick_left_h
  if (lhs->stick_left_h != rhs->stick_left_h) {
    return false;
  }
  // var_a
  if (lhs->var_a != rhs->var_a) {
    return false;
  }
  return true;
}

bool
scout_msgs__msg__ScoutRCState__copy(
  const scout_msgs__msg__ScoutRCState * input,
  scout_msgs__msg__ScoutRCState * output)
{
  if (!input || !output) {
    return false;
  }
  // swa
  output->swa = input->swa;
  // swb
  output->swb = input->swb;
  // swc
  output->swc = input->swc;
  // swd
  output->swd = input->swd;
  // stick_right_v
  output->stick_right_v = input->stick_right_v;
  // stick_right_h
  output->stick_right_h = input->stick_right_h;
  // stick_left_v
  output->stick_left_v = input->stick_left_v;
  // stick_left_h
  output->stick_left_h = input->stick_left_h;
  // var_a
  output->var_a = input->var_a;
  return true;
}

scout_msgs__msg__ScoutRCState *
scout_msgs__msg__ScoutRCState__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutRCState * msg = (scout_msgs__msg__ScoutRCState *)allocator.allocate(sizeof(scout_msgs__msg__ScoutRCState), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(scout_msgs__msg__ScoutRCState));
  bool success = scout_msgs__msg__ScoutRCState__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
scout_msgs__msg__ScoutRCState__destroy(scout_msgs__msg__ScoutRCState * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    scout_msgs__msg__ScoutRCState__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
scout_msgs__msg__ScoutRCState__Sequence__init(scout_msgs__msg__ScoutRCState__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutRCState * data = NULL;

  if (size) {
    data = (scout_msgs__msg__ScoutRCState *)allocator.zero_allocate(size, sizeof(scout_msgs__msg__ScoutRCState), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = scout_msgs__msg__ScoutRCState__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        scout_msgs__msg__ScoutRCState__fini(&data[i - 1]);
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
scout_msgs__msg__ScoutRCState__Sequence__fini(scout_msgs__msg__ScoutRCState__Sequence * array)
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
      scout_msgs__msg__ScoutRCState__fini(&array->data[i]);
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

scout_msgs__msg__ScoutRCState__Sequence *
scout_msgs__msg__ScoutRCState__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  scout_msgs__msg__ScoutRCState__Sequence * array = (scout_msgs__msg__ScoutRCState__Sequence *)allocator.allocate(sizeof(scout_msgs__msg__ScoutRCState__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = scout_msgs__msg__ScoutRCState__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
scout_msgs__msg__ScoutRCState__Sequence__destroy(scout_msgs__msg__ScoutRCState__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    scout_msgs__msg__ScoutRCState__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
scout_msgs__msg__ScoutRCState__Sequence__are_equal(const scout_msgs__msg__ScoutRCState__Sequence * lhs, const scout_msgs__msg__ScoutRCState__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!scout_msgs__msg__ScoutRCState__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
scout_msgs__msg__ScoutRCState__Sequence__copy(
  const scout_msgs__msg__ScoutRCState__Sequence * input,
  scout_msgs__msg__ScoutRCState__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(scout_msgs__msg__ScoutRCState);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    scout_msgs__msg__ScoutRCState * data =
      (scout_msgs__msg__ScoutRCState *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!scout_msgs__msg__ScoutRCState__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          scout_msgs__msg__ScoutRCState__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!scout_msgs__msg__ScoutRCState__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
