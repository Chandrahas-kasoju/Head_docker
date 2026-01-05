// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarClassifier.idl
// generated code does not contain a copyright notice
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_classifier__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__init(ti_mmwave_rospkg_msgs__msg__RadarClassifier * msg)
{
  if (!msg) {
    return false;
  }
  // track_id
  // value
  return true;
}

void
ti_mmwave_rospkg_msgs__msg__RadarClassifier__fini(ti_mmwave_rospkg_msgs__msg__RadarClassifier * msg)
{
  if (!msg) {
    return;
  }
  // track_id
  // value
}

bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarClassifier * lhs, const ti_mmwave_rospkg_msgs__msg__RadarClassifier * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // track_id
  if (lhs->track_id != rhs->track_id) {
    return false;
  }
  // value
  if (lhs->value != rhs->value) {
    return false;
  }
  return true;
}

bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarClassifier * input,
  ti_mmwave_rospkg_msgs__msg__RadarClassifier * output)
{
  if (!input || !output) {
    return false;
  }
  // track_id
  output->track_id = input->track_id;
  // value
  output->value = input->value;
  return true;
}

ti_mmwave_rospkg_msgs__msg__RadarClassifier *
ti_mmwave_rospkg_msgs__msg__RadarClassifier__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarClassifier * msg = (ti_mmwave_rospkg_msgs__msg__RadarClassifier *)allocator.allocate(sizeof(ti_mmwave_rospkg_msgs__msg__RadarClassifier), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ti_mmwave_rospkg_msgs__msg__RadarClassifier));
  bool success = ti_mmwave_rospkg_msgs__msg__RadarClassifier__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ti_mmwave_rospkg_msgs__msg__RadarClassifier__destroy(ti_mmwave_rospkg_msgs__msg__RadarClassifier * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ti_mmwave_rospkg_msgs__msg__RadarClassifier__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__init(ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarClassifier * data = NULL;

  if (size) {
    data = (ti_mmwave_rospkg_msgs__msg__RadarClassifier *)allocator.zero_allocate(size, sizeof(ti_mmwave_rospkg_msgs__msg__RadarClassifier), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ti_mmwave_rospkg_msgs__msg__RadarClassifier__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ti_mmwave_rospkg_msgs__msg__RadarClassifier__fini(&data[i - 1]);
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
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__fini(ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * array)
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
      ti_mmwave_rospkg_msgs__msg__RadarClassifier__fini(&array->data[i]);
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

ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence *
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * array = (ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence *)allocator.allocate(sizeof(ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__destroy(ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * lhs, const ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ti_mmwave_rospkg_msgs__msg__RadarClassifier__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * input,
  ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ti_mmwave_rospkg_msgs__msg__RadarClassifier);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ti_mmwave_rospkg_msgs__msg__RadarClassifier * data =
      (ti_mmwave_rospkg_msgs__msg__RadarClassifier *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ti_mmwave_rospkg_msgs__msg__RadarClassifier__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ti_mmwave_rospkg_msgs__msg__RadarClassifier__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ti_mmwave_rospkg_msgs__msg__RadarClassifier__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
