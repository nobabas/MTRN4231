// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from blue_detector_msgs:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_
#define BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "blue_detector_msgs/msg/detail/marker_data__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace blue_detector_msgs
{

namespace msg
{

namespace builder
{

class Init_MarkerData_pose
{
public:
  explicit Init_MarkerData_pose(::blue_detector_msgs::msg::MarkerData & msg)
  : msg_(msg)
  {}
  ::blue_detector_msgs::msg::MarkerData pose(::blue_detector_msgs::msg::MarkerData::_pose_type arg)
  {
    msg_.pose = std::move(arg);
    return std::move(msg_);
  }

private:
  ::blue_detector_msgs::msg::MarkerData msg_;
};

class Init_MarkerData_id
{
public:
  Init_MarkerData_id()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MarkerData_pose id(::blue_detector_msgs::msg::MarkerData::_id_type arg)
  {
    msg_.id = std::move(arg);
    return Init_MarkerData_pose(msg_);
  }

private:
  ::blue_detector_msgs::msg::MarkerData msg_;
};

}  // namespace builder

}  // namespace msg

template<typename MessageType>
auto build();

template<>
inline
auto build<::blue_detector_msgs::msg::MarkerData>()
{
  return blue_detector_msgs::msg::builder::Init_MarkerData_id();
}

}  // namespace blue_detector_msgs

#endif  // BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__BUILDER_HPP_
