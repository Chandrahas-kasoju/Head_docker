// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__BUILDER_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_contents__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarMicroDopplerFeatureContents_s_entropy
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_s_entropy(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents s_entropy(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_s_entropy_type arg)
  {
    msg_.s_entropy = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_med_freq
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_med_freq(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureContents_s_entropy med_freq(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_med_freq_type arg)
  {
    msg_.med_freq = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_s_entropy(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_mean_freq
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_mean_freq(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureContents_med_freq mean_freq(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_mean_freq_type arg)
  {
    msg_.mean_freq = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_med_freq(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_bw_pwr
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_bw_pwr(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureContents_mean_freq bw_pwr(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_bw_pwr_type arg)
  {
    msg_.bw_pwr = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_mean_freq(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_f_up
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_f_up(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureContents_bw_pwr f_up(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_f_up_type arg)
  {
    msg_.f_up = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_bw_pwr(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_f_low
{
public:
  explicit Init_RadarMicroDopplerFeatureContents_f_low(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureContents_f_up f_low(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_f_low_type arg)
  {
    msg_.f_low = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_f_up(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

class Init_RadarMicroDopplerFeatureContents_header
{
public:
  Init_RadarMicroDopplerFeatureContents_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarMicroDopplerFeatureContents_f_low header(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarMicroDopplerFeatureContents_f_low(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents>()
{
  return ti_mmwave_rospkg_msgs::msg::builder::Init_RadarMicroDopplerFeatureContents_header();
}

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__BUILDER_HPP_
