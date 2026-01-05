// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__FUNCTIONS_H_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "posture_analysis_msgs/msg/rosidl_generator_c__visibility_control.h"

#include "posture_analysis_msgs/msg/detail/posture__struct.h"

/// Initialize msg/Posture message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * posture_analysis_msgs__msg__Posture
 * )) before or use
 * posture_analysis_msgs__msg__Posture__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__init(posture_analysis_msgs__msg__Posture * msg);

/// Finalize msg/Posture message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
void
posture_analysis_msgs__msg__Posture__fini(posture_analysis_msgs__msg__Posture * msg);

/// Create msg/Posture message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * posture_analysis_msgs__msg__Posture__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
posture_analysis_msgs__msg__Posture *
posture_analysis_msgs__msg__Posture__create();

/// Destroy msg/Posture message.
/**
 * It calls
 * posture_analysis_msgs__msg__Posture__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
void
posture_analysis_msgs__msg__Posture__destroy(posture_analysis_msgs__msg__Posture * msg);

/// Check for msg/Posture message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__are_equal(const posture_analysis_msgs__msg__Posture * lhs, const posture_analysis_msgs__msg__Posture * rhs);

/// Copy a msg/Posture message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__copy(
  const posture_analysis_msgs__msg__Posture * input,
  posture_analysis_msgs__msg__Posture * output);

/// Initialize array of msg/Posture messages.
/**
 * It allocates the memory for the number of elements and calls
 * posture_analysis_msgs__msg__Posture__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__Sequence__init(posture_analysis_msgs__msg__Posture__Sequence * array, size_t size);

/// Finalize array of msg/Posture messages.
/**
 * It calls
 * posture_analysis_msgs__msg__Posture__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
void
posture_analysis_msgs__msg__Posture__Sequence__fini(posture_analysis_msgs__msg__Posture__Sequence * array);

/// Create array of msg/Posture messages.
/**
 * It allocates the memory for the array and calls
 * posture_analysis_msgs__msg__Posture__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
posture_analysis_msgs__msg__Posture__Sequence *
posture_analysis_msgs__msg__Posture__Sequence__create(size_t size);

/// Destroy array of msg/Posture messages.
/**
 * It calls
 * posture_analysis_msgs__msg__Posture__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
void
posture_analysis_msgs__msg__Posture__Sequence__destroy(posture_analysis_msgs__msg__Posture__Sequence * array);

/// Check for msg/Posture message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__Sequence__are_equal(const posture_analysis_msgs__msg__Posture__Sequence * lhs, const posture_analysis_msgs__msg__Posture__Sequence * rhs);

/// Copy an array of msg/Posture messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_posture_analysis_msgs
bool
posture_analysis_msgs__msg__Posture__Sequence__copy(
  const posture_analysis_msgs__msg__Posture__Sequence * input,
  posture_analysis_msgs__msg__Posture__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__FUNCTIONS_H_
