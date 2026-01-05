// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from posture_analysis_msgs:msg/Posture.idl
// generated code does not contain a copyright notice

#ifndef POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_HPP_
#define POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__posture_analysis_msgs__msg__Posture __attribute__((deprecated))
#else
# define DEPRECATED__posture_analysis_msgs__msg__Posture __declspec(deprecated)
#endif

namespace posture_analysis_msgs
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct Posture_
{
  using Type = Posture_<ContainerAllocator>;

  explicit Posture_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->posture_class = 0l;
      this->uprightness_score = 0l;
    }
  }

  explicit Posture_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->posture_class = 0l;
      this->uprightness_score = 0l;
    }
  }

  // field types and members
  using _posture_class_type =
    int32_t;
  _posture_class_type posture_class;
  using _uprightness_score_type =
    int32_t;
  _uprightness_score_type uprightness_score;

  // setters for named parameter idiom
  Type & set__posture_class(
    const int32_t & _arg)
  {
    this->posture_class = _arg;
    return *this;
  }
  Type & set__uprightness_score(
    const int32_t & _arg)
  {
    this->uprightness_score = _arg;
    return *this;
  }

  // constant declarations
  static constexpr int32_t UNKNOWN =
    0;
  static constexpr int32_t LYING_DOWN =
    1;
  static constexpr int32_t SITTING =
    2;
  static constexpr int32_t STANDING =
    3;
  static constexpr int32_t UPRIGHT =
    3;

  // pointer types
  using RawPtr =
    posture_analysis_msgs::msg::Posture_<ContainerAllocator> *;
  using ConstRawPtr =
    const posture_analysis_msgs::msg::Posture_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      posture_analysis_msgs::msg::Posture_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      posture_analysis_msgs::msg::Posture_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__posture_analysis_msgs__msg__Posture
    std::shared_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__posture_analysis_msgs__msg__Posture
    std::shared_ptr<posture_analysis_msgs::msg::Posture_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const Posture_ & other) const
  {
    if (this->posture_class != other.posture_class) {
      return false;
    }
    if (this->uprightness_score != other.uprightness_score) {
      return false;
    }
    return true;
  }
  bool operator!=(const Posture_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct Posture_

// alias to use template instance with default allocator
using Posture =
  posture_analysis_msgs::msg::Posture_<std::allocator<void>>;

// constant definitions
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Posture_<ContainerAllocator>::UNKNOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Posture_<ContainerAllocator>::LYING_DOWN;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Posture_<ContainerAllocator>::SITTING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Posture_<ContainerAllocator>::STANDING;
#endif  // __cplusplus < 201703L
#if __cplusplus < 201703L
// static constexpr member variable definitions are only needed in C++14 and below, deprecated in C++17
template<typename ContainerAllocator>
constexpr int32_t Posture_<ContainerAllocator>::UPRIGHT;
#endif  // __cplusplus < 201703L

}  // namespace msg

}  // namespace posture_analysis_msgs

#endif  // POSTURE_ANALYSIS_MSGS__MSG__DETAIL__POSTURE__STRUCT_HPP_
