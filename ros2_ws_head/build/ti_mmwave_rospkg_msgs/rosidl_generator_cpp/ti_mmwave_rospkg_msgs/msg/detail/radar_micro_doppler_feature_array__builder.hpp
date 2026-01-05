// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureArray.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__BUILDER_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_feature_array__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarMicroDopplerFeatureArray_track
{
public:
  explicit Init_RadarMicroDopplerFeatureArray_track(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray & msg)
  : msg_(msg)
  {}
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray track(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray::_track_type arg)
  {
    msg_.track = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray msg_;
};

class Init_RadarMicroDopplerFeatureArray_num_tracks
{
public:
  explicit Init_RadarMicroDopplerFeatureArray_num_tracks(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray & msg)
  : msg_(msg)
  {}
  Init_RadarMicroDopplerFeatureArray_track num_tracks(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray::_num_tracks_type arg)
  {
    msg_.num_tracks = std::move(arg);
    return Init_RadarMicroDopplerFeatureArray_track(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray msg_;
};

class Init_RadarMicroDopplerFeatureArray_header
{
public:
  Init_RadarMicroDopplerFeatureArray_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarMicroDopplerFeatureArray_num_tracks header(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarMicroDopplerFeatureArray_num_tracks(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureArray>()
{
  return ti_mmwave_rospkg_msgs::msg::builder::Init_RadarMicroDopplerFeatureArray_header();
}

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_ARRAY__BUILDER_HPP_
