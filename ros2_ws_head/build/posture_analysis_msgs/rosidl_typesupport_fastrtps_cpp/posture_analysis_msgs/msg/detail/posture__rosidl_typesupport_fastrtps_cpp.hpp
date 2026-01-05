// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__rosidl_typesupport_fastrtps_cpp.hpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_

#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_interface/macros.h"
#include "posture_analysis_msgs/msg/rosidl_typesupport_fastrtps_cpp__visibility_control.h"
#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"

#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-parameter"
# ifdef __clang__
#  pragma clang diagnostic ignored "-Wdeprecated-register"
#  pragma clang diagnostic ignored "-Wreturn-type-c-linkage"
# endif
#endif
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif

#include "fastcdr/Cdr.h"

namespace posture_analysis_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
cdr_serialize(
  const posture_analysis_msgs::msg::Posture & ros_message,
  eprosima::fastcdr::Cdr & cdr);

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  posture_analysis_msgs::msg::Posture & ros_message);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
get_serialized_size(
  const posture_analysis_msgs::msg::Posture & ros_message,
  size_t current_alignment);

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
max_serialized_size_Posture(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace posture_analysis_msgs

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, posture_analysis_msgs, msg, Posture)();

#ifdef __cplusplus
}
#endif

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__ROSIDL_TYPESUPPORT_FASTRTPS_CPP_HPP_
