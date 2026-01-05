// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice
#include "posture_analysis_msgs/msg/detail/posture__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
posture_analysis_msgs__msg__Posture__init(posture_analysis_msgs__msg__Posture * msg)
{
  if (!msg) {
    return false;
  }
  // posture_class
  // uprightness_score
  return true;
}

void
posture_analysis_msgs__msg__Posture__fini(posture_analysis_msgs__msg__Posture * msg)
{
  if (!msg) {
    return;
  }
  // posture_class
  // uprightness_score
}

bool
posture_analysis_msgs__msg__Posture__are_equal(const posture_analysis_msgs__msg__Posture * lhs, const posture_analysis_msgs__msg__Posture * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // posture_class
  if (lhs->posture_class != rhs->posture_class) {
    return false;
  }
  // uprightness_score
  if (lhs->uprightness_score != rhs->uprightness_score) {
    return false;
  }
  return true;
}

bool
posture_analysis_msgs__msg__Posture__copy(
  const posture_analysis_msgs__msg__Posture * input,
  posture_analysis_msgs__msg__Posture * output)
{
  if (!input || !output) {
    return false;
  }
  // posture_class
  output->posture_class = input->posture_class;
  // uprightness_score
  output->uprightness_score = input->uprightness_score;
  return true;
}

posture_analysis_msgs__msg__Posture *
posture_analysis_msgs__msg__Posture__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  posture_analysis_msgs__msg__Posture * msg = (posture_analysis_msgs__msg__Posture *)allocator.allocate(sizeof(posture_analysis_msgs__msg__Posture), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(posture_analysis_msgs__msg__Posture));
  bool success = posture_analysis_msgs__msg__Posture__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
posture_analysis_msgs__msg__Posture__destroy(posture_analysis_msgs__msg__Posture * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    posture_analysis_msgs__msg__Posture__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
posture_analysis_msgs__msg__Posture__Sequence__init(posture_analysis_msgs__msg__Posture__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  posture_analysis_msgs__msg__Posture * data = NULL;

  if (size) {
    data = (posture_analysis_msgs__msg__Posture *)allocator.zero_allocate(size, sizeof(posture_analysis_msgs__msg__Posture), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = posture_analysis_msgs__msg__Posture__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        posture_analysis_msgs__msg__Posture__fini(&data[i - 1]);
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
posture_analysis_msgs__msg__Posture__Sequence__fini(posture_analysis_msgs__msg__Posture__Sequence * array)
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
      posture_analysis_msgs__msg__Posture__fini(&array->data[i]);
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

posture_analysis_msgs__msg__Posture__Sequence *
posture_analysis_msgs__msg__Posture__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  posture_analysis_msgs__msg__Posture__Sequence * array = (posture_analysis_msgs__msg__Posture__Sequence *)allocator.allocate(sizeof(posture_analysis_msgs__msg__Posture__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = posture_analysis_msgs__msg__Posture__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
posture_analysis_msgs__msg__Posture__Sequence__destroy(posture_analysis_msgs__msg__Posture__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    posture_analysis_msgs__msg__Posture__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
posture_analysis_msgs__msg__Posture__Sequence__are_equal(const posture_analysis_msgs__msg__Posture__Sequence * lhs, const posture_analysis_msgs__msg__Posture__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!posture_analysis_msgs__msg__Posture__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
posture_analysis_msgs__msg__Posture__Sequence__copy(
  const posture_analysis_msgs__msg__Posture__Sequence * input,
  posture_analysis_msgs__msg__Posture__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(posture_analysis_msgs__msg__Posture);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    posture_analysis_msgs__msg__Posture * data =
      (posture_analysis_msgs__msg__Posture *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!posture_analysis_msgs__msg__Posture__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          posture_analysis_msgs__msg__Posture__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!posture_analysis_msgs__msg__Posture__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
