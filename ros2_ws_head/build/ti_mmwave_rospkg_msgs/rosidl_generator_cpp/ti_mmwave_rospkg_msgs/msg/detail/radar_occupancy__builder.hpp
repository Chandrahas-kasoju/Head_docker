// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarOccupancy.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__BUILDER_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "ti_mmwave_rospkg_msgs/msg/detail/radar_occupancy__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

namespace builder
{

class Init_RadarOccupancy_state
{
public:
  explicit Init_RadarOccupancy_state(::ti_mmwave_rospkg_msgs::msg::RadarOccupancy & msg)
  : msg_(msg)
  {}
  ::ti_mmwave_rospkg_msgs::msg::RadarOccupancy state(::ti_mmwave_rospkg_msgs::msg::RadarOccupancy::_state_type arg)
  {
    msg_.state = std::move(arg);
    return std::move(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarOccupancy msg_;
};

class Init_RadarOccupancy_header
{
public:
  Init_RadarOccupancy_header()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_RadarOccupancy_state header(::ti_mmwave_rospkg_msgs::msg::RadarOccupancy::_header_type arg)
  {
    msg_.header = std::move(arg);
    return Init_RadarOccupancy_state(msg_);
  }

private:
  ::ti_mmwave_rospkg_msgs::msg::RadarOccupancy msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::ti_mmwave_rospkg_msgs::msg::RadarOccupancy>()
{
  return ti_mmwave_rospkg_msgs::msg::builder::Init_RadarOccupancy_header();
}

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__BUILDER_HPP_
