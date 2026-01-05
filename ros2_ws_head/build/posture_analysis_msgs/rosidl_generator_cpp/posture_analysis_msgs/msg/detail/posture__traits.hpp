// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__TRAITS_HPP_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace posture_analysis_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const Posture & msg,
  std::ostream & out)
{
  out << "{";
  // member: posture_class
  {
    out << "posture_class: ";
    rosidl_generator_traits::value_to_yaml(msg.posture_class, out);
    out << ", ";
  }

  // member: uprightness_score
  {
    out << "uprightness_score: ";
    rosidl_generator_traits::value_to_yaml(msg.uprightness_score, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const Posture & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: posture_class
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "posture_class: ";
    rosidl_generator_traits::value_to_yaml(msg.posture_class, out);
    out << "\n";
  }

  // member: uprightness_score
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "uprightness_score: ";
    rosidl_generator_traits::value_to_yaml(msg.uprightness_score, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const Posture & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace posture_analysis_msgs

namespace rosidl_generator_traits
{

[[deprecated("use posture_analysis_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const posture_analysis_msgs::msg::Posture & msg,
  std::ostream & out, size_t indentation = 0)
{
  posture_analysis_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use posture_analysis_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const posture_analysis_msgs::msg::Posture & msg)
{
  return posture_analysis_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<posture_analysis_msgs::msg::Posture>()
{
  return "posture_analysis_msgs::msg::Posture";
}

template<>
inline const char * name<posture_analysis_msgs::msg::Posture>()
{
  return "posture_analysis_msgs/msg/Posture";
}

template<>
struct has_fixed_size<posture_analysis_msgs::msg::Posture>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<posture_analysis_msgs::msg::Posture>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<posture_analysis_msgs::msg::Posture>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__TRAITS_HPP_
