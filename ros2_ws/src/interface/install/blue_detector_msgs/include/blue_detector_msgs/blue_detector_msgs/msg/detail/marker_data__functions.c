// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from blue_detector_msgs:msg/MarkerData.idl
// generated code does not contain a copyright notice
#include "blue_detector_msgs/msg/detail/marker_data__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `pose`
#include "rosidl_runtime_c/primitives_sequence_functions.h"

bool
blue_detector_msgs__msg__MarkerData__init(blue_detector_msgs__msg__MarkerData * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // pose
  if (!rosidl_runtime_c__float__Sequence__init(&msg->pose, 0)) {
    blue_detector_msgs__msg__MarkerData__fini(msg);
    return false;
  }
  return true;
}

void
blue_detector_msgs__msg__MarkerData__fini(blue_detector_msgs__msg__MarkerData * msg)
{
  if (!msg) {
    return;
  }
  // id
  // pose
  rosidl_runtime_c__float__Sequence__fini(&msg->pose);
}

bool
blue_detector_msgs__msg__MarkerData__are_equal(const blue_detector_msgs__msg__MarkerData * lhs, const blue_detector_msgs__msg__MarkerData * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // pose
  if (!rosidl_runtime_c__float__Sequence__are_equal(
      &(lhs->pose), &(rhs->pose)))
  {
    return false;
  }
  return true;
}

bool
blue_detector_msgs__msg__MarkerData__copy(
  const blue_detector_msgs__msg__MarkerData * input,
  blue_detector_msgs__msg__MarkerData * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // pose
  if (!rosidl_runtime_c__float__Sequence__copy(
      &(input->pose), &(output->pose)))
  {
    return false;
  }
  return true;
}

blue_detector_msgs__msg__MarkerData *
blue_detector_msgs__msg__MarkerData__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blue_detector_msgs__msg__MarkerData * msg = (blue_detector_msgs__msg__MarkerData *)allocator.allocate(sizeof(blue_detector_msgs__msg__MarkerData), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(blue_detector_msgs__msg__MarkerData));
  bool success = blue_detector_msgs__msg__MarkerData__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
blue_detector_msgs__msg__MarkerData__destroy(blue_detector_msgs__msg__MarkerData * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    blue_detector_msgs__msg__MarkerData__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
blue_detector_msgs__msg__MarkerData__Sequence__init(blue_detector_msgs__msg__MarkerData__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blue_detector_msgs__msg__MarkerData * data = NULL;

  if (size) {
    data = (blue_detector_msgs__msg__MarkerData *)allocator.zero_allocate(size, sizeof(blue_detector_msgs__msg__MarkerData), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = blue_detector_msgs__msg__MarkerData__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        blue_detector_msgs__msg__MarkerData__fini(&data[i - 1]);
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
blue_detector_msgs__msg__MarkerData__Sequence__fini(blue_detector_msgs__msg__MarkerData__Sequence * array)
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
      blue_detector_msgs__msg__MarkerData__fini(&array->data[i]);
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

blue_detector_msgs__msg__MarkerData__Sequence *
blue_detector_msgs__msg__MarkerData__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  blue_detector_msgs__msg__MarkerData__Sequence * array = (blue_detector_msgs__msg__MarkerData__Sequence *)allocator.allocate(sizeof(blue_detector_msgs__msg__MarkerData__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = blue_detector_msgs__msg__MarkerData__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
blue_detector_msgs__msg__MarkerData__Sequence__destroy(blue_detector_msgs__msg__MarkerData__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    blue_detector_msgs__msg__MarkerData__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
blue_detector_msgs__msg__MarkerData__Sequence__are_equal(const blue_detector_msgs__msg__MarkerData__Sequence * lhs, const blue_detector_msgs__msg__MarkerData__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!blue_detector_msgs__msg__MarkerData__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
blue_detector_msgs__msg__MarkerData__Sequence__copy(
  const blue_detector_msgs__msg__MarkerData__Sequence * input,
  blue_detector_msgs__msg__MarkerData__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(blue_detector_msgs__msg__MarkerData);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    blue_detector_msgs__msg__MarkerData * data =
      (blue_detector_msgs__msg__MarkerData *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!blue_detector_msgs__msg__MarkerData__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          blue_detector_msgs__msg__MarkerData__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!blue_detector_msgs__msg__MarkerData__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
