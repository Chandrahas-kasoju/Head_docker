// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_H_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Constant 'UNKNOWN'.
/**
  * Posture Classes (Constants for clarity)
 */
enum
{
  posture_analysis_msgs__msg__Posture__UNKNOWN = 0l
};

/// Constant 'LYING_DOWN'.
enum
{
  posture_analysis_msgs__msg__Posture__LYING_DOWN = 1l
};

/// Constant 'SITTING'.
enum
{
  posture_analysis_msgs__msg__Posture__SITTING = 2l
};

/// Constant 'STANDING'.
enum
{
  posture_analysis_msgs__msg__Posture__STANDING = 3l
};

/// Constant 'UPRIGHT'.
enum
{
  posture_analysis_msgs__msg__Posture__UPRIGHT = 3l
};

/// Struct defined in msg/Posture in the package posture_analysis_msgs.
/**
  * Posture.msg
  * Defines the detected human posture with an integer class and a score.
 */
typedef struct posture_analysis_msgs__msg__Posture
{
  /// The detected posture class, using one of the constants above
  int32_t posture_class;
  /// A score from 0-100 indicating uprightness
  int32_t uprightness_score;
} posture_analysis_msgs__msg__Posture;

// Struct for a sequence of posture_analysis_msgs__msg__Posture.
typedef struct posture_analysis_msgs__msg__Posture__Sequence
{
  posture_analysis_msgs__msg__Posture * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} posture_analysis_msgs__msg__Posture__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_H_
