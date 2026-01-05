// generated from rosidl_typesupport_cpp/resource/idl__type_support.cpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#include "cstddef"
#include "rosidl_runtime_c/message_type_support_struct.h"
#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"
#include "rosidl_typesupport_cpp/identifier.hpp"
#include "rosidl_typesupport_cpp/message_type_support.hpp"
#include "rosidl_typesupport_c/type_support_map.h"
#include "rosidl_typesupport_cpp/message_type_support_dispatch.hpp"
#include "rosidl_typesupport_cpp/visibility_control.h"
#include "rosidl_typesupport_interface/macros.h"

namespace posture_analysis_msgs
{

namespace msg
{

namespace rosidl_typesupport_cpp
{

typedef struct _Posture_type_support_ids_t
{
  const char * typesupport_identifier[2];
} _Posture_type_support_ids_t;

static const _Posture_type_support_ids_t _Posture_message_typesupport_ids = {
  {
    "rosidl_typesupport_fastrtps_cpp",  // ::rosidl_typesupport_fastrtps_cpp::typesupport_identifier,
    "rosidl_typesupport_introspection_cpp",  // ::rosidl_typesupport_introspection_cpp::typesupport_identifier,
  }
};

typedef struct _Posture_type_support_symbol_names_t
{
  const char * symbol_name[2];
} _Posture_type_support_symbol_names_t;

#define STRINGIFY_(s) #s
#define STRINGIFY(s) STRINGIFY_(s)

static const _Posture_type_support_symbol_names_t _Posture_message_typesupport_symbol_names = {
  {
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_fastrtps_cpp, posture_analysis_msgs, msg, Posture)),
    STRINGIFY(ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_introspection_cpp, posture_analysis_msgs, msg, Posture)),
  }
};

typedef struct _Posture_type_support_data_t
{
  void * data[2];
} _Posture_type_support_data_t;

static _Posture_type_support_data_t _Posture_message_typesupport_data = {
  {
    0,  // will store the shared library later
    0,  // will store the shared library later
  }
};

static const type_support_map_t _Posture_message_typesupport_map = {
  2,
  "posture_analysis_msgs",
  &_Posture_message_typesupport_ids.typesupport_identifier[0],
  &_Posture_message_typesupport_symbol_names.symbol_name[0],
  &_Posture_message_typesupport_data.data[0],
};

static const rosidl_message_type_support_t Posture_message_type_support_handle = {
  ::rosidl_typesupport_cpp::typesupport_identifier,
  reinterpret_cast<const type_support_map_t *>(&_Posture_message_typesupport_map),
  ::rosidl_typesupport_cpp::get_message_typesupport_handle_function,
};

}  // namespace rosidl_typesupport_cpp

}  // namespace msg

}  // namespace posture_analysis_msgs

namespace rosidl_typesupport_cpp
{

template<>
ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
get_message_type_support_handle<posture_analysis_msgs::msg::Posture>()
{
  return &::posture_analysis_msgs::msg::rosidl_typesupport_cpp::Posture_message_type_support_handle;
}

#ifdef __cplusplus
extern "C"
{
#endif

ROSIDL_TYPESUPPORT_CPP_PUBLIC
const rosidl_message_type_support_t *
ROSIDL_TYPESUPPORT_INTERFACE__MESSAGE_SYMBOL_NAME(rosidl_typesupport_cpp, posture_analysis_msgs, msg, Posture)() {
  return get_message_type_support_handle<posture_analysis_msgs::msg::Posture>();
}

#ifdef __cplusplus
}
#endif
}  // namespace rosidl_typesupport_cpp
