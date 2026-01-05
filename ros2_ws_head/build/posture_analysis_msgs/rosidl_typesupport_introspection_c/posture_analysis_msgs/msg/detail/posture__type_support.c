// generated from rosidl_typesupport_introspection_c/resource/idl__type_support.c.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#include <stddef.h>
#include "posture_analysis_msgs/msg/detail/posture__rosidl_typesupport_introspection_c.h"
#include "posture_analysis_msgs/msg/rosidl_typesupport_introspection_c__visibility_control.h"
#include "rosidl_typesupport_introspection_c/field_types.h"
#include "rosidl_typesupport_introspection_c/identifier.h"
#include "rosidl_typesupport_introspection_c/message_introspection.h"
#include "posture_analysis_msgs/msg/detail/posture__functions.h"
#include "posture_analysis_msgs/msg/detail/posture__struct.h"


#ifdef __cplusplus
extern "C"
{
#endif

void posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_init_function(
  void * message_memory, enum rosidl_runtime_c__message_initialization _init)
{
  // TODO(karsten1987): initializers are not yet implemented for typesupport c
  // see https://github.com/ros2/ros2/issues/397
  (void) _init;
  posture_analysis_msgs__msg__Posture__init(message_memory);
}

void posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_fini_function(void * message_memory)
{
  posture_analysis_msgs__msg__Posture__fini(message_memory);
}

static rosidl_typesupport_introspection_c__MessageMember posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_member_array[2] = {
  {
    "posture_class",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(posture_analysis_msgs__msg__Posture, posture_class),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  },
  {
    "uprightness_score",  // name
    rosidl_typesupport_introspection_c__ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    NULL,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(posture_analysis_msgs__msg__Posture, uprightness_score),  // bytes offset in struct
    NULL,  // default value
    NULL,  // size() function pointer
    NULL,  // get_const(index) function pointer
    NULL,  // get(index) function pointer
    NULL,  // fetch(index, &value) function pointer
    NULL,  // assign(index, value) function pointer
    NULL  // resize(index) function pointer
  }
};

static const rosidl_typesupport_introspection_c__MessageMembers posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_members = {
  "posture_analysis_msgs__msg",  // message namespace
  "Posture",  // message name
  2,  // number of fields
  sizeof(posture_analysis_msgs__msg__Posture),
  posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_member_array,  // message members
  posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_init_function,  // function to initialize message memory (memory has to be allocated)
  posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_fini_function  // function to terminate message instance (will not free memory)
};

// this is not const since it must be initialized on first access
// since C does not allow non-integral compile-time constants
static rosidl_message_type_support_t posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_type_support_handle = {
  0,
  &posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_members,
  get_message_typesupport_handle_function,
};

ROSIDL_TYPESUPPORT_INTROSPECTION_C_EXPORT_posture_analysis_msgs
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_c, posture_analysis_msgs, msg, Posture)() {
  if (!posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_type_support_handle.typesupport_identifier) {
    posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_type_support_handle.typesupport_identifier =
      rosidl_typesupport_introspection_c__identifier;
  }
  return &posture_analysis_msgs__msg__Posture__rosidl_typesupport_introspection_c__Posture_message_type_support_handle;
}
#ifdef __cplusplus
}
#endif
