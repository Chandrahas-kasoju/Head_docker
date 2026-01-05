// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
// generated code does not contain a copyright notice
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `header`
#include "std_msgs/msg/detail/header__functions.h"

bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__init(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * msg)
{
  if (!msg) {
    return false;
  }
  // header
  if (!std_msgs__msg__Header__init(&msg->header)) {
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(msg);
    return false;
  }
  // f_low
  // f_up
  // bw_pwr
  // mean_freq
  // med_freq
  // s_entropy
  return true;
}

void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * msg)
{
  if (!msg) {
    return;
  }
  // header
  std_msgs__msg__Header__fini(&msg->header);
  // f_low
  // f_up
  // bw_pwr
  // mean_freq
  // med_freq
  // s_entropy
}

bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * lhs, const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * rhs)
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
  // f_low
  if (lhs->f_low != rhs->f_low) {
    return false;
  }
  // f_up
  if (lhs->f_up != rhs->f_up) {
    return false;
  }
  // bw_pwr
  if (lhs->bw_pwr != rhs->bw_pwr) {
    return false;
  }
  // mean_freq
  if (lhs->mean_freq != rhs->mean_freq) {
    return false;
  }
  // med_freq
  if (lhs->med_freq != rhs->med_freq) {
    return false;
  }
  // s_entropy
  if (lhs->s_entropy != rhs->s_entropy) {
    return false;
  }
  return true;
}

bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * input,
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * output)
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
  // f_low
  output->f_low = input->f_low;
  // f_up
  output->f_up = input->f_up;
  // bw_pwr
  output->bw_pwr = input->bw_pwr;
  // mean_freq
  output->mean_freq = input->mean_freq;
  // med_freq
  output->med_freq = input->med_freq;
  // s_entropy
  output->s_entropy = input->s_entropy;
  return true;
}

ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents *
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * msg = (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents *)allocator.allocate(sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents));
  bool success = ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__destroy(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__init(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * data = NULL;

  if (size) {
    data = (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents *)allocator.zero_allocate(size, sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(&data[i - 1]);
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
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__fini(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * array)
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
      ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(&array->data[i]);
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

ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence *
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * array = (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence *)allocator.allocate(sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__destroy(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__are_equal(const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * lhs, const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence__copy(
  const ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * input,
  ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents * data =
      (ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
