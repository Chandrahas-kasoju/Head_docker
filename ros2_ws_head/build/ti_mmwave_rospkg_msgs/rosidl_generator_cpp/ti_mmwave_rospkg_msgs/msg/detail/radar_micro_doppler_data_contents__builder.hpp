// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__BUILDER_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_contents__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarMicroDopplerDataContents_value
{
public:
  Init_RadarMicroDopplerDataContents_value()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents value(::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents::_value_type arg)
  {
    msg_.value = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents>()
{
  return ti_mmwave_rospkg_msgs::msg::builder::Init_RadarMicroDopplerDataContents_value();
}

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__BUILDER_HPP_
