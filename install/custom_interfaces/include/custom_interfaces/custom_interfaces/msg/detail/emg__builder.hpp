// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from custom_interfaces:msg/EMG.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__EMG__BUILDER_HPP_
#define CUSTOM_INTERFACES__MSG__DETAIL__EMG__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "custom_interfaces/msg/detail/emg__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace custom_interfaces
{

namespace msg
{

namespace builder
{

class Init_EMG_ch2
{
public:
  explicit Init_EMG_ch2(::custom_interfaces::msg::EMG & msg)
  : msg_(msg)
  {}
  ::custom_interfaces::msg::EMG ch2(::custom_interfaces::msg::EMG::_ch2_type arg)
  {
    msg_.ch2 = std::move(arg);
    return std::move(msg_);
  }

private:
  ::custom_interfaces::msg::EMG msg_;
};

class Init_EMG_ch1
{
public:
  Init_EMG_ch1()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_EMG_ch2 ch1(::custom_interfaces::msg::EMG::_ch1_type arg)
  {
    msg_.ch1 = std::move(arg);
    return Init_EMG_ch2(msg_);
  }

private:
  ::custom_interfaces::msg::EMG msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::custom_interfaces::msg::EMG>()
{
  return custom_interfaces::msg::builder::Init_EMG_ch1();
}

}  // namespace custom_interfaces

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__EMG__BUILDER_HPP_
