// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interface:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_
#define INTERFACE__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interface/msg/detail/marker_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interface
{

namespace msg
{

namespace builder
{

class Init_MarkerData_pose
{
public:
  explicit Init_MarkerData_pose(::interface::msg::MarkerData & msg)
  : msg_(msg)
  {}
  ::interface::msg::MarkerData pose(::interface::msg::MarkerData::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interface::msg::MarkerData msg_;
};

class Init_MarkerData_id
{
public:
  Init_MarkerData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarkerData_pose id(::interface::msg::MarkerData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MarkerData_pose(msg_);
  }

private:
  ::interface::msg::MarkerData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::interface::msg::MarkerData>()
{
  return interface::msg::builder::Init_MarkerData_id();
}

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_
