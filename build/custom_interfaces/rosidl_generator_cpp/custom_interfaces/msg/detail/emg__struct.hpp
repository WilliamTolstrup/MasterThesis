// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from custom_interfaces:msg/EMG.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__custom_interfaces__msg__EMG __attribute__((deprecated))
#else
# define DEPRECATED__custom_interfaces__msg__EMG __declspec(deprecated)
#endif

namespace custom_interfaces
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct EMG_
{
  using Type = EMG_<ContainerAllocator>;

  explicit EMG_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ch1 = 0.0f;
      this->ch2 = 0.0f;
    }
  }

  explicit EMG_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->ch1 = 0.0f;
      this->ch2 = 0.0f;
    }
  }

  // field types and members
  using _ch1_type =
    float;
  _ch1_type ch1;
  using _ch2_type =
    float;
  _ch2_type ch2;

  // setters for named parameter idiom
  Type & set__ch1(
    const float & _arg)
  {
    this->ch1 = _arg;
    return *this;
  }
  Type & set__ch2(
    const float & _arg)
  {
    this->ch2 = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    custom_interfaces::msg::EMG_<ContainerAllocator> *;
  using ConstRawPtr =
    const custom_interfaces::msg::EMG_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<custom_interfaces::msg::EMG_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<custom_interfaces::msg::EMG_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::EMG_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::EMG_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      custom_interfaces::msg::EMG_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<custom_interfaces::msg::EMG_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<custom_interfaces::msg::EMG_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<custom_interfaces::msg::EMG_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__custom_interfaces__msg__EMG
    std::shared_ptr<custom_interfaces::msg::EMG_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__custom_interfaces__msg__EMG
    std::shared_ptr<custom_interfaces::msg::EMG_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const EMG_ & other) const
  {
    if (this->ch1 != other.ch1) {
      return false;
    }
    if (this->ch2 != other.ch2) {
      return false;
    }
    return true;
  }
  bool operator!=(const EMG_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct EMG_

// alias to use template instance with default allocator
using EMG =
  custom_interfaces::msg::EMG_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_HPP_
