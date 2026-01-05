// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarClassifier.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__STRUCT_H_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RadarClassifier in the package ti_mmwave_rospkg_msgs.
typedef struct ti_mmwave_rospkg_msgs__msg__RadarClassifier
{
  uint16_t track_id;
  float value;
} ti_mmwave_rospkg_msgs__msg__RadarClassifier;

// Struct for a sequence of ti_mmwave_rospkg_msgs__msg__RadarClassifier.
typedef struct ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence
{
  ti_mmwave_rospkg_msgs__msg__RadarClassifier * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} ti_mmwave_rospkg_msgs__msg__RadarClassifier__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_CLASSIFIER__STRUCT_H_
