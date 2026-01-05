// generated from rosidl_typesupport_introspection_cpp/resource/idl__type_support.cpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#include "array"
#include "cstddef"
#include "string"
#include "vector"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_interface/macros.h"
#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"
#include "rosidl_typesupport_introspection_cpp/field_types.hpp"
#include "rosidl_typesupport_introspection_cpp/identifier.hpp"
#include "rosidl_typesupport_introspection_cpp/message_introspection.hpp"
#include "rosidl_typesupport_introspection_cpp/message_type_support_decl.hpp"
#include "rosidl_typesupport_introspection_cpp/visibility_control.h"

namespace posture_analysis_msgs
{

namespace msg
{

namespace rosidl_typesupport_introspection_cpp
{

void Posture_init_function(
  void * message_memory, rosidl_runtime_cpp::MessageInitialization _init)
{
  new (message_memory) posture_analysis_msgs::msg::Posture(_init);
}

void Posture_fini_function(void * message_memory)
{
  auto typed_message = static_cast<posture_analysis_msgs::msg::Posture *>(message_memory);
  typed_message->~Posture();
}

static const ::rosidl_typesupport_introspection_cpp::MessageMember Posture_message_member_array[2] = {
  {
    "posture_class",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(posture_analysis_msgs::msg::Posture, posture_class),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  },
  {
    "uprightness_score",  // name
    ::rosidl_typesupport_introspection_cpp::ROS_TYPE_INT32,  // type
    0,  // upper bound of string
    nullptr,  // members of sub message
    false,  // is array
    0,  // array size
    false,  // is upper bound
    offsetof(posture_analysis_msgs::msg::Posture, uprightness_score),  // bytes offset in struct
    nullptr,  // default value
    nullptr,  // size() function pointer
    nullptr,  // get_const(index) function pointer
    nullptr,  // get(index) function pointer
    nullptr,  // fetch(index, &value) function pointer
    nullptr,  // assign(index, value) function pointer
    nullptr  // resize(index) function pointer
  }
};

static const ::rosidl_typesupport_introspection_cpp::MessageMembers Posture_message_members = {
  "posture_analysis_msgs::msg",  // message namespace
  "Posture",  // message name
  2,  // number of fields
  sizeof(posture_analysis_msgs::msg::Posture),
  Posture_message_member_array,  // message members
  Posture_init_function,  // function to initialize message memory (memory has to be allocated)
  Posture_fini_function  // function to terminate message instance (will not free memory)
};

static const rosidl_message_type_support_t Posture_message_type_support_handle = {
  ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  &Posture_message_members,
  get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_introspection_cpp

}  // namespace msg

}  // namespace posture_analysis_msgs


namespace rosidl_typesupport_introspection_cpp
{

template<>
ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<posture_analysis_msgs::msg::Posture>()
{
  return &::posture_analysis_msgs::msg::rosidl_typesupport_introspection_cpp::Posture_message_type_support_handle;
}

}  // namespace rosidl_typesupport_introspection_cpp

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_INTROSPECTION_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, posture_analysis_msgs, msg, Posture)() {
  return &::posture_analysis_msgs::msg::rosidl_typesupport_introspection_cpp::Posture_message_type_support_handle;
}

#ifdef __cplusplus
}
#endif
