// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice
#include "posture_analysis_msgs/msg/detail/posture__rosidl_typesupport_fastrtps_cpp.hpp"
#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"

#include <limits>
#include <stdexcept>
#include <string>
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_fastrtps_cpp/identifier.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_fastrtps_cpp/wstring_conversion.hpp"
#include "fastcdr/Cdr.h"


// forward declaration of message dependencies and their conversion functions

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
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: posture_class
  cdr << ros_message.posture_class;
  // Member: uprightness_score
  cdr << ros_message.uprightness_score;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  posture_analysis_msgs::msg::Posture & ros_message)
{
  // Member: posture_class
  cdr >> ros_message.posture_class;

  // Member: uprightness_score
  cdr >> ros_message.uprightness_score;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
get_serialized_size(
  const posture_analysis_msgs::msg::Posture & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: posture_class
  {
    size_t item_size = sizeof(ros_message.posture_class);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: uprightness_score
  {
    size_t item_size = sizeof(ros_message.uprightness_score);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_posture_analysis_msgs
max_serialized_size_Posture(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  size_t last_member_size = 0;
  (void)last_member_size;
  (void)padding;
  (void)wchar_size;

  full_bounded = true;
  is_plain = true;


  // Member: posture_class
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: uprightness_score
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  size_t ret_val = current_alignment - initial_alignment;
  if (is_plain) {
    // All members are plain, and type is not empty.
    // We still need to check that the in-memory alignment
    // is the same as the CDR mandated alignment.
    using DataType = posture_analysis_msgs::msg::Posture;
    is_plain =
      (
      offsetof(DataType, uprightness_score) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _Posture__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const posture_analysis_msgs::msg::Posture *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _Posture__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<posture_analysis_msgs::msg::Posture *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _Posture__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const posture_analysis_msgs::msg::Posture *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _Posture__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_Posture(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _Posture__callbacks = {
  "posture_analysis_msgs::msg",
  "Posture",
  _Posture__cdr_serialize,
  _Posture__cdr_deserialize,
  _Posture__get_serialized_size,
  _Posture__max_serialized_size
};

static rosidl_message_type_support_t _Posture__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_Posture__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace posture_analysis_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_posture_analysis_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<posture_analysis_msgs::msg::Posture>()
{
  return &posture_analysis_msgs::msg::typesupport_fastrtps_cpp::_Posture__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, posture_analysis_msgs, msg, Posture)() {
  return &posture_analysis_msgs::msg::typesupport_fastrtps_cpp::_Posture__handle;
}

#ifdef __cplusplus
}
#endif
