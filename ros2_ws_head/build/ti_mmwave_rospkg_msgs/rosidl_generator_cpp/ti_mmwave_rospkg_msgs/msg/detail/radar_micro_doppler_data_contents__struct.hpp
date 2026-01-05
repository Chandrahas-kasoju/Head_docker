// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataContents.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__STRUCT_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents __attribute__((deprecated))
#else
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents __declspec(deprecated)
#endif

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarMicroDopplerDataContents_
{
  using Type = RadarMicroDopplerDataContents_<ContainerAllocator>;

  explicit RadarMicroDopplerDataContents_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0.0f;
    }
  }

  explicit RadarMicroDopplerDataContents_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->value = 0.0f;
    }
  }

  // field types and members
  using _value_type =
    float;
  _value_type value;

  // setters for named parameter idiom
  Type & set__value(
    const float & _arg)
  {
    this->value = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> *;
  using ConstRawPtr =
    const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataContents
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarMicroDopplerDataContents_ & other) const
  {
    if (this->value != other.value) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarMicroDopplerDataContents_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarMicroDopplerDataContents_

// alias to use template instance with default allocator
using RadarMicroDopplerDataContents =
  ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_CONTENTS__STRUCT_HPP_
