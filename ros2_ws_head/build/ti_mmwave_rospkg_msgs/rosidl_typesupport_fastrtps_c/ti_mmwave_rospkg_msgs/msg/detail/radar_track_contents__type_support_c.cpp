// generated from rosidl_typesupport_fastrtps_c/resource/idl__type_support_c.cpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarTrackContents.idl
// generated code does not contain a copyright notice
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_track_contents__rosidl_typesupport_fastrtps_c.h"


#include <cassert>
#include <limits>
#include <string>
#include "rosidl_typesupport_fastrtps_c/identifier.h"
#include "rosidl_typesupport_fastrtps_c/wstring_conversion.hpp"
#include "rosidl_typesupport_fastrtps_cpp/message_type_support.h"
#include "ti_mmwave_rospkg_msgs/msg/rosidl_typesupport_fastrtps_c__visibility_control.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_track_contents__struct.h"
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_track_contents__functions.h"
#include "fastcdr/Cdr.h"

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

// includes and forward declarations of message dependencies and their conversion functions

#if defined(__cplusplus)
extern "C"
{
#endif

#include "std_msgs/msg/detail/header__functions.h"  // header

// forward declare type support functions
ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ti_mmwave_rospkg_msgs
size_t get_serialized_size_std_msgs__msg__Header(
  const void * untyped_ros_message,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ti_mmwave_rospkg_msgs
size_t max_serialized_size_std_msgs__msg__Header(
  bool & full_bounded,
  bool & is_plain,
  size_t current_alignment);

ROSIDL_TYPESUPPORT_FASTRTPS_C_IMPORT_ti_mmwave_rospkg_msgs
const rosidl_message_type_support_t *
  ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, std_msgs, msg, Header)();


using _RadarTrackContents__ros_msg_type = ti_mmwave_rospkg_msgs__msg__RadarTrackContents;

static bool _RadarTrackContents__cdr_serialize(
  const void * untyped_ros_message,
  eprosima::fastcdr::Cdr & cdr)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  const _RadarTrackContents__ros_msg_type * ros_message = static_cast<const _RadarTrackContents__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_serialize(
        &ros_message->header, cdr))
    {
      return false;
    }
  }

  // Field name: tid
  {
    cdr << ros_message->tid;
  }

  // Field name: posx
  {
    cdr << ros_message->posx;
  }

  // Field name: posy
  {
    cdr << ros_message->posy;
  }

  // Field name: posz
  {
    cdr << ros_message->posz;
  }

  // Field name: velx
  {
    cdr << ros_message->velx;
  }

  // Field name: vely
  {
    cdr << ros_message->vely;
  }

  // Field name: velz
  {
    cdr << ros_message->velz;
  }

  // Field name: accx
  {
    cdr << ros_message->accx;
  }

  // Field name: accy
  {
    cdr << ros_message->accy;
  }

  // Field name: accz
  {
    cdr << ros_message->accz;
  }

  return true;
}

static bool _RadarTrackContents__cdr_deserialize(
  eprosima::fastcdr::Cdr & cdr,
  void * untyped_ros_message)
{
  if (!untyped_ros_message) {
    fprintf(stderr, "ros message handle is null\n");
    return false;
  }
  _RadarTrackContents__ros_msg_type * ros_message = static_cast<_RadarTrackContents__ros_msg_type *>(untyped_ros_message);
  // Field name: header
  {
    const message_type_support_callbacks_t * callbacks =
      static_cast<const message_type_support_callbacks_t *>(
      ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(
        rosidl_typesupport_fastrtps_c, std_msgs, msg, Header
      )()->data);
    if (!callbacks->cdr_deserialize(
        cdr, &ros_message->header))
    {
      return false;
    }
  }

  // Field name: tid
  {
    cdr >> ros_message->tid;
  }

  // Field name: posx
  {
    cdr >> ros_message->posx;
  }

  // Field name: posy
  {
    cdr >> ros_message->posy;
  }

  // Field name: posz
  {
    cdr >> ros_message->posz;
  }

  // Field name: velx
  {
    cdr >> ros_message->velx;
  }

  // Field name: vely
  {
    cdr >> ros_message->vely;
  }

  // Field name: velz
  {
    cdr >> ros_message->velz;
  }

  // Field name: accx
  {
    cdr >> ros_message->accx;
  }

  // Field name: accy
  {
    cdr >> ros_message->accy;
  }

  // Field name: accz
  {
    cdr >> ros_message->accz;
  }

  return true;
}  // NOLINT(readability/fn_size)

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ti_mmwave_rospkg_msgs
size_t get_serialized_size_ti_mmwave_rospkg_msgs__msg__RadarTrackContents(
  const void * untyped_ros_message,
  size_t current_alignment)
{
  const _RadarTrackContents__ros_msg_type * ros_message = static_cast<const _RadarTrackContents__ros_msg_type *>(untyped_ros_message);
  (void)ros_message;
  size_t initial_alignment = current_alignment;

  const size_t padding = 4;
  const size_t wchar_size = 4;
  (void)padding;
  (void)wchar_size;

  // field.name header

  current_alignment += get_serialized_size_std_msgs__msg__Header(
    &(ros_message->header), current_alignment);
  // field.name tid
  {
    size_t item_size = sizeof(ros_message->tid);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name posx
  {
    size_t item_size = sizeof(ros_message->posx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name posy
  {
    size_t item_size = sizeof(ros_message->posy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name posz
  {
    size_t item_size = sizeof(ros_message->posz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velx
  {
    size_t item_size = sizeof(ros_message->velx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name vely
  {
    size_t item_size = sizeof(ros_message->vely);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name velz
  {
    size_t item_size = sizeof(ros_message->velz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name accx
  {
    size_t item_size = sizeof(ros_message->accx);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name accy
  {
    size_t item_size = sizeof(ros_message->accy);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }
  // field.name accz
  {
    size_t item_size = sizeof(ros_message->accz);
    current_alignment += item_size +
      eprosima::fastcdr::Cdr::alignment(current_alignment, item_size);
  }

  return current_alignment - initial_alignment;
}

static uint32_t _RadarTrackContents__get_serialized_size(const void * untyped_ros_message)
{
  return static_cast<uint32_t>(
    get_serialized_size_ti_mmwave_rospkg_msgs__msg__RadarTrackContents(
      untyped_ros_message, 0));
}

ROSIDL_TYPESUPPORT_FASTRTPS_C_PUBLIC_ti_mmwave_rospkg_msgs
size_t max_serialized_size_ti_mmwave_rospkg_msgs__msg__RadarTrackContents(
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

  // member: header
  {
    size_t array_size = 1;


    last_member_size = 0;
    for (size_t index = 0; index < array_size; ++index) {
      bool inner_full_bounded;
      bool inner_is_plain;
      size_t inner_size;
      inner_size =
        max_serialized_size_std_msgs__msg__Header(
        inner_full_bounded, inner_is_plain, current_alignment);
      last_member_size += inner_size;
      current_alignment += inner_size;
      full_bounded &= inner_full_bounded;
      is_plain &= inner_is_plain;
    }
  }
  // member: tid
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: posx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: posy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: posz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: vely
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: velz
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: accx
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: accy
  {
    size_t array_size = 1;

    last_member_size = array_size * sizeof(uint32_t);
    current_alignment += array_size * sizeof(uint32_t) +
      eprosima::fastcdr::Cdr::alignment(current_alignment, sizeof(uint32_t));
  }
  // member: accz
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
    using DataType = ti_mmwave_rospkg_msgs__msg__RadarTrackContents;
    is_plain =
      (
      offsetof(DataType, accz) +
      last_member_size
      ) == ret_val;
  }

  return ret_val;
}

static size_t _RadarTrackContents__max_serialized_size(char & bounds_info)
{
  bool full_bounded;
  bool is_plain;
  size_t ret_val;

  ret_val = max_serialized_size_ti_mmwave_rospkg_msgs__msg__RadarTrackContents(
    full_bounded, is_plain, 0);

  bounds_info =
    is_plain ? ROSIDL_TYPESUPPORT_FASTRTPS_PLAIN_TYPE :
    full_bounded ? ROSIDL_TYPESUPPORT_FASTRTPS_BOUNDED_TYPE : ROSIDL_TYPESUPPORT_FASTRTPS_UNBOUNDED_TYPE;
  return ret_val;
}


static message_type_support_callbacks_t __callbacks_RadarTrackContents = {
  "ti_mmwave_rospkg_msgs::msg",
  "RadarTrackContents",
  _RadarTrackContents__cdr_serialize,
  _RadarTrackContents__cdr_deserialize,
  _RadarTrackContents__get_serialized_size,
  _RadarTrackContents__max_serialized_size
};

static rosidl_message_type_support_t _RadarTrackContents__type_support = {
  rosidl_typesupport_fastrtps_c__identifier,
  &__callbacks_RadarTrackContents,
  get_message_typesupport_handle_function,
};

const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_c, ti_mmwave_rospkg_msgs, msg, RadarTrackContents)() {
  return &_RadarTrackContents__type_support;
}

#if defined(__cplusplus)
}
#endif
