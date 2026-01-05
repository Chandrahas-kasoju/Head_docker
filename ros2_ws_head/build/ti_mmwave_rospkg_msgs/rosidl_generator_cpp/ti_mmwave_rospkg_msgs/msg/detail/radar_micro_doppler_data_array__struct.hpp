// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from ti_mmwave_rospkg_msgs:msg/RadarMicroDopplerDataArray.idl
// generated code does not contain a copyright notice

#ifndef TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_ARRAY__STRUCT_HPP_
#define TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_ARRAY__STRUCT_HPP_

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
// Member 'track'
#include "ti_mmwave_rospkg_msgs/msg/detail/radar_micro_doppler_data_contents__struct.hpp"

#ifndef _WIN32
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray __attribute__((deprecated))
#else
# define DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray __declspec(deprecated)
#endif

namespace ti_mmwave_rospkg_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct RadarMicroDopplerDataArray_
{
  using Type = RadarMicroDopplerDataArray_<ContainerAllocator>;

  explicit RadarMicroDopplerDataArray_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_tracks = 0ul;
    }
  }

  explicit RadarMicroDopplerDataArray_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  : header(_alloc, _init)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->num_tracks = 0ul;
    }
  }

  // field types and members
  using _header_type =
    std_msgs::msg::Header_<ContainerAllocator>;
  _header_type header;
  using _num_tracks_type =
    uint32_t;
  _num_tracks_type num_tracks;
  using _track_type =
    std::vector<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>>;
  _track_type track;

  // setters for named parameter idiom
  Type & set__header(
    const std_msgs::msg::Header_<ContainerAllocator> & _arg)
  {
    this->header = _arg;
    return *this;
  }
  Type & set__num_tracks(
    const uint32_t & _arg)
  {
    this->num_tracks = _arg;
    return *this;
  }
  Type & set__track(
    const std::vector<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataContents_<ContainerAllocator>>> & _arg)
  {
    this->track = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> *;
  using ConstRawPtr =
    const ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__ti_mmwave_rospkg_msgs__msg__RadarMicroDopplerDataArray
    std::shared_ptr<ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const RadarMicroDopplerDataArray_ & other) const
  {
    if (this->header != other.header) {
      return false;
    }
    if (this->num_tracks != other.num_tracks) {
      return false;
    }
    if (this->track != other.track) {
      return false;
    }
    return true;
  }
  bool operator!=(const RadarMicroDopplerDataArray_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct RadarMicroDopplerDataArray_

// alias to use template instance with default allocator
using RadarMicroDopplerDataArray =
  ti_mmwave_rospkg_msgs::msg::RadarMicroDopplerDataArray_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace ti_mmwave_rospkg_msgs

#endif  // TI_MMWAVE_ROSPKG_MSGS__MSG__DETAIL__RADAR_MICRO_DOPPLER_DATA_ARRAY__STRUCT_HPP_
