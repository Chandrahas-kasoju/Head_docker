// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarOccupancy.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__STRUCT_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__STRUCT_HPP_

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
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarOccupancy __attribute__((deprecated))
#else
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarOccupancy __declspec(deprecated)
#endif

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarOccupancy_
{
  using Type = RadarOccupancy_<ContainerAllocator>;

  explicit RadarOccupancy_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0ul;
    }
  }

  explicit RadarOccupancy_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->state = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _state_type =
    uint32_t;
  _state_type state;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__state(
    const uint32_t & _arg)
  {
    this->state = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> *;
  using ConstRawPtr =
    const ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarOccupancy
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarOccupancy
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarOccupancy_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->state != other.state) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarOccupancy_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarOccupancy_

// alias to use template instance with default allocator
using RadarOccupancy =
  ti_mmwave_rospkg_msgs::msg::RadarOccupancy_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_OCCUPANCY__STRUCT_HPP_
