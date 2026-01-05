// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__TRAITS_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_contents__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarMicroDopplerDataContents & msg,
  std::ostream & out)
{
  out << "{";
  // member: value
  {
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadarMicroDopplerDataContents & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: value
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "value: ";
    rosidl_generator_traits::value_to_yaml(msg.value, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarMicroDopplerDataContents & msg, bool use_flow_style = false)
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

}  // namespace ti_mmwave_rospkg_msgs

namespace rosidl_generator_traits
{

[[deprecated("use ti_mmwave_rospkg_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents & msg,
  std::ostream & out, size_t indentation = 0)
{
  ti_mmwave_rospkg_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ti_mmwave_rospkg_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents & msg)
{
  return ti_mmwave_rospkg_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>()
{
  return "ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents";
}

template<>
inline const char * name<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>()
{
  return "ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerDataContents";
}

template<>
struct has_fixed_size<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>
  : std::integral_constant<bool, true> {};

template<>
struct has_bounded_size<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>
  : std::integral_constant<bool, true> {};

template<>
struct is_message<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__TRAITS_HPP_
