// generated from rosidl_typesupport_fastrtps_cpp/resource/idl__type_support.cpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
// generated code does not contain a copyright notice
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__rosidl_typesupport_fastrtps_cpp.hpp"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__struct.hpp"

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
namespace std_msgs
{
namespace msg
{
namespace typesupport_fastrtps_cpp
{
bool cdr_serialize(
  const std_msgs::msg::Header &,
  eprosima::fastcdr::Cdr &);
bool cdr_deserialize(
  eprosima::fastcdr::Cdr &,
  std_msgs::msg::Header &);
size_t get_serialized_size(
  const std_msgs::msg::Header &,
  size_t current_alignment);
size_t
max_serialized_size_Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);
}  // namespace typesupport_fastrtps_cpp
}  // namespace msg
}  // namespace std_msgs


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace typesupport_fastrtps_cpp
{

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ti_mmwave_rospkg_msgs
cdr_serialize(
  const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_serialize(
    ros_message.header,
    cdr);
  // Member: f_low
  cdr << ros_message.f_low;
  // Member: f_up
  cdr << ros_message.f_up;
  // Member: bw_pwr
  cdr << ros_message.bw_pwr;
  // Member: mean_freq
  cdr << ros_message.mean_freq;
  // Member: med_freq
  cdr << ros_message.med_freq;
  // Member: s_entropy
  cdr << ros_message.s_entropy;
  return true;
}

bool
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ti_mmwave_rospkg_msgs
cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & ros_message)
{
  // Member: header
  std_msgs::msg::typesupport_fastrtps_cpp::cdr_deserialize(
    cdr, ros_message.header);

  // Member: f_low
  cdr >> ros_message.f_low;

  // Member: f_up
  cdr >> ros_message.f_up;

  // Member: bw_pwr
  cdr >> ros_message.bw_pwr;

  // Member: mean_freq
  cdr >> ros_message.mean_freq;

  // Member: med_freq
  cdr >> ros_message.med_freq;

  // Member: s_entropy
  cdr >> ros_message.s_entropy;

  return true;
}  // NOLINT(readability/fn_size)

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ti_mmwave_rospkg_msgs
get_serialized_size(
  const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & ros_message,
  size_t current_alignment)
{
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // Member: header

  current_alignment +=
    std_msgs::msg::typesupport_fastrtps_cpp::get_serialized_size(
    ros_message.header, current_alignment);
  // Member: f_low
  {
    size_t item_size = sizeof(ros_message.f_low);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: f_up
  {
    size_t item_size = sizeof(ros_message.f_up);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: bw_pwr
  {
    size_t item_size = sizeof(ros_message.bw_pwr);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: mean_freq
  {
    size_t item_size = sizeof(ros_message.mean_freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: med_freq
  {
    size_t item_size = sizeof(ros_message.med_freq);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // Member: s_entropy
  {
    size_t item_size = sizeof(ros_message.s_entropy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

size_t
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_ti_mmwave_rospkg_msgs
max_serialized_size_RadarMicroDopplerFeatureContents(
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


  // Member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size =
        std_msgs::msg::typesupport_fastrtps_cpp::max_serialized_size_Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }

  // Member: f_low
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: f_up
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: bw_pwr
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: mean_freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: med_freq
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }

  // Member: s_entropy
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
    using DataType = ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents;
    is_plain =
      (
      offsetof(DataType, s_entropy) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static bool _RadarMicroDopplerFeatureContents__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  auto typed_message =
    static_cast<const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents *>(
    untyped_ros_message);
  return cdr_serialize(*typed_message, cdr);
}

static bool _RadarMicroDopplerFeatureContents__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  auto typed_message =
    static_cast<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents *>(
    untyped_ros_message);
  return cdr_deserialize(cdr, *typed_message);
}

static uint32_t _RadarMicroDopplerFeatureContents__get_serialized_size(
  const void * untyped_ros_message)
{
  auto typed_message =
    static_cast<const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents *>(
    untyped_ros_message);
  return static_cast<uint32_t>(get_serialized_size(*typed_message, 0));
}

static size_t _RadarMicroDopplerFeatureContents__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_RadarMicroDopplerFeatureContents(full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}

static message_type_support_callbacks_t _RadarMicroDopplerFeatureContents__callbacks = {
  "ti_mmwave_rospkg_msgs::msg",
  "RadarMicroDopplerFeatureContents",
  _RadarMicroDopplerFeatureContents__cdr_serialize,
  _RadarMicroDopplerFeatureContents__cdr_deserialize,
  _RadarMicroDopplerFeatureContents__get_serialized_size,
  _RadarMicroDopplerFeatureContents__max_serialized_size
};

static rosidl_message_type_support_t _RadarMicroDopplerFeatureContents__handle = {
  rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
  &_RadarMicroDopplerFeatureContents__callbacks,
  get_message_typesupport_handle_function,
};

}  // namespace typesupport_fastrtps_cpp

}  // namespace msg

}  // namespace ti_mmwave_rospkg_msgs

namespace rosidl_typesupport_fastrtps_cpp
{

template<>
ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_ti_mmwave_rospkg_msgs
const rosidl_message_type_support_t *
get_message_type_support_handle<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>()
{
  return &ti_mmwave_rospkg_msgs::msg::typesupport_fastrtps_cpp::_RadarMicroDopplerFeatureContents__handle;
}

}  // namespace rosidl_typesupport_fastrtps_cpp

#ifdef __cplusplus
extern "C"
{
#endif

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, ti_mmwave_rospkg_msgs, msg, RadarMicroDopplerFeatureContents)() {
  return &ti_mmwave_rospkg_msgs::msg::typesupport_fastrtps_cpp::_RadarMicroDopplerFeatureContents__handle;
}

#ifdef __cplusplus
}
#endif
