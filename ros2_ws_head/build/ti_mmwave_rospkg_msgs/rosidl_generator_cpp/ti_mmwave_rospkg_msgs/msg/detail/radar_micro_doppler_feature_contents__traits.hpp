// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__TRAITS_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__traits.hpp"

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const RadarMicroDopplerFeatureContents & msg,
  std::ostream & out)
{
  out << "{";
  // member: header
  {
    out << "header: ";
    to_flow_style_yaml(msg.header, out);
    out << ", ";
  }

  // member: f_low
  {
    out << "f_low: ";
    rosidl_generator_traits::value_to_yaml(msg.f_low, out);
    out << ", ";
  }

  // member: f_up
  {
    out << "f_up: ";
    rosidl_generator_traits::value_to_yaml(msg.f_up, out);
    out << ", ";
  }

  // member: bw_pwr
  {
    out << "bw_pwr: ";
    rosidl_generator_traits::value_to_yaml(msg.bw_pwr, out);
    out << ", ";
  }

  // member: mean_freq
  {
    out << "mean_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.mean_freq, out);
    out << ", ";
  }

  // member: med_freq
  {
    out << "med_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.med_freq, out);
    out << ", ";
  }

  // member: s_entropy
  {
    out << "s_entropy: ";
    rosidl_generator_traits::value_to_yaml(msg.s_entropy, out);
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const RadarMicroDopplerFeatureContents & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: header
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "header:\n";
    to_block_style_yaml(msg.header, out, indentation + 2);
  }

  // member: f_low
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f_low: ";
    rosidl_generator_traits::value_to_yaml(msg.f_low, out);
    out << "\n";
  }

  // member: f_up
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "f_up: ";
    rosidl_generator_traits::value_to_yaml(msg.f_up, out);
    out << "\n";
  }

  // member: bw_pwr
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "bw_pwr: ";
    rosidl_generator_traits::value_to_yaml(msg.bw_pwr, out);
    out << "\n";
  }

  // member: mean_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "mean_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.mean_freq, out);
    out << "\n";
  }

  // member: med_freq
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "med_freq: ";
    rosidl_generator_traits::value_to_yaml(msg.med_freq, out);
    out << "\n";
  }

  // member: s_entropy
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "s_entropy: ";
    rosidl_generator_traits::value_to_yaml(msg.s_entropy, out);
    out << "\n";
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const RadarMicroDopplerFeatureContents & msg, bool use_flow_style = false)
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
  const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg,
  std::ostream & out, size_t indentation = 0)
{
  ti_mmwave_rospkg_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use ti_mmwave_rospkg_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
{
  return ti_mmwave_rospkg_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>()
{
  return "ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents";
}

template<>
inline const char * name<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>()
{
  return "ti_mmwave_rospkg_msgs/msg/RadarMicroDopplerFeatureContents";
}

template<>
struct has_fixed_size<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>
  : std::integral_constant<bool, has_fixed_size<std_msgs::msg::Header>::value> {};

template<>
struct has_bounded_size<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>
  : std::integral_constant<bool, has_bounded_size<std_msgs::msg::Header>::value> {};

template<>
struct is_message<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__TRAITS_HPP_
