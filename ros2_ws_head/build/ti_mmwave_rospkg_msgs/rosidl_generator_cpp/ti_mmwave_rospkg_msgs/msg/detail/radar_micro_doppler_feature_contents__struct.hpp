// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerFeatureContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__STRUCT_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents __attribute__((deprecated))
#else
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents __declspec(deprecated)
#endif

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarMicroDopplerFeatureContents_
{
  using Type = RadarMicroDopplerFeatureContents_<ContainerAllocator>;

  explicit RadarMicroDopplerFeatureContents_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f_low = 0.0f;
      this->f_up = 0.0f;
      this->bw_pwr = 0.0f;
      this->mean_freq = 0.0f;
      this->med_freq = 0.0f;
      this->s_entropy = 0.0f;
    }
  }

  explicit RadarMicroDopplerFeatureContents_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->f_low = 0.0f;
      this->f_up = 0.0f;
      this->bw_pwr = 0.0f;
      this->mean_freq = 0.0f;
      this->med_freq = 0.0f;
      this->s_entropy = 0.0f;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _f_low_type =
    float;
  _f_low_type f_low;
  using _f_up_type =
    float;
  _f_up_type f_up;
  using _bw_pwr_type =
    float;
  _bw_pwr_type bw_pwr;
  using _mean_freq_type =
    float;
  _mean_freq_type mean_freq;
  using _med_freq_type =
    float;
  _med_freq_type med_freq;
  using _s_entropy_type =
    float;
  _s_entropy_type s_entropy;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__f_low(
    const float & _arg)
  {
    this->f_low = _arg;
    return *this;
  }
  Type & set__f_up(
    const float & _arg)
  {
    this->f_up = _arg;
    return *this;
  }
  Type & set__bw_pwr(
    const float & _arg)
  {
    this->bw_pwr = _arg;
    return *this;
  }
  Type & set__mean_freq(
    const float & _arg)
  {
    this->mean_freq = _arg;
    return *this;
  }
  Type & set__med_freq(
    const float & _arg)
  {
    this->med_freq = _arg;
    return *this;
  }
  Type & set__s_entropy(
    const float & _arg)
  {
    this->s_entropy = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> *;
  using ConstRawPtr =
    const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerFeatureContents
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarMicroDopplerFeatureContents_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->f_low != other.f_low) {
      return false;
    }
    if (this->f_up != other.f_up) {
      return false;
    }
    if (this->bw_pwr != other.bw_pwr) {
      return false;
    }
    if (this->mean_freq != other.mean_freq) {
      return false;
    }
    if (this->med_freq != other.med_freq) {
      return false;
    }
    if (this->s_entropy != other.s_entropy) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarMicroDopplerFeatureContents_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarMicroDopplerFeatureContents_

// alias to use template instance with default allocator
using RadarMicroDopplerFeatureContents =
  ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerFeatureContents_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_FEATURE_CONTENTS__STRUCT_HPP_
