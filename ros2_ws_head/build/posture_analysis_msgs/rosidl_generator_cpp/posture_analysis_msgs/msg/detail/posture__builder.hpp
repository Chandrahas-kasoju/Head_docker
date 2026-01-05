// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__BUILDER_HPP_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "posture_analysis_msgs/msg/detail/posture__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace posture_analysis_msgs
{

namespace msg
{

namespace builder
{

class Init_Posture_uprightness_score
{
public:
  explicit Init_Posture_uprightness_score(::posture_analysis_msgs::msg::Posture & msg)
  : msg_(msg)
  {}
  ::posture_analysis_msgs::msg::Posture uprightness_score(::posture_analysis_msgs::msg::Posture::_uprightness_score_type arg)
  {
    msg_.uprightness_score = std::move(arg);
    return std::move(msg_);
  }

private:
  ::posture_analysis_msgs::msg::Posture msg_;
};

class Init_Posture_posture_class
{
public:
  Init_Posture_posture_class()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_Posture_uprightness_score posture_class(::posture_analysis_msgs::msg::Posture::_posture_class_type arg)
  {
    msg_.posture_class = std::move(arg);
    return Init_Posture_uprightness_score(msg_);
  }

private:
  ::posture_analysis_msgs::msg::Posture msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::posture_analysis_msgs::msg::Posture>()
{
  return posture_analysis_msgs::msg::builder::Init_Posture_posture_class();
}

}  // namespace posture_analysis_msgs

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__BUILDER_HPP_
