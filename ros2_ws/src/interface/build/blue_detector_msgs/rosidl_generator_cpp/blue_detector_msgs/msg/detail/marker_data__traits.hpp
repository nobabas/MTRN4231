// generated from rosidl_generator_cpp/resource/idl__traits.hpp.em
// with input from blue_detector_msgs:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__TRAITS_HPP_
#define BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__TRAITS_HPP_

#include <stdint.h>

#include <sstream>
#include <string>
#include <type_traits>

#include "blue_detector_msgs/msg/detail/marker_data__struct.hpp"
#include "rosidl_runtime_cpp/traits.hpp"

namespace blue_detector_msgs
{

namespace msg
{

inline void to_flow_style_yaml(
  const MarkerData & msg,
  std::ostream & out)
{
  out << "{";
  // member: id
  {
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << ", ";
  }

  // member: pose
  {
    if (msg.pose.size() == 0) {
      out << "pose: []";
    } else {
      out << "pose: [";
      size_t pending_items = msg.pose.size();
      for (auto item : msg.pose) {
        rosidl_generator_traits::value_to_yaml(item, out);
        if (--pending_items > 0) {
          out << ", ";
        }
      }
      out << "]";
    }
  }
  out << "}";
}  // NOLINT(readability/fn_size)

inline void to_block_style_yaml(
  const MarkerData & msg,
  std::ostream & out, size_t indentation = 0)
{
  // member: id
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    out << "id: ";
    rosidl_generator_traits::value_to_yaml(msg.id, out);
    out << "\n";
  }

  // member: pose
  {
    if (indentation > 0) {
      out << std::string(indentation, ' ');
    }
    if (msg.pose.size() == 0) {
      out << "pose: []\n";
    } else {
      out << "pose:\n";
      for (auto item : msg.pose) {
        if (indentation > 0) {
          out << std::string(indentation, ' ');
        }
        out << "- ";
        rosidl_generator_traits::value_to_yaml(item, out);
        out << "\n";
      }
    }
  }
}  // NOLINT(readability/fn_size)

inline std::string to_yaml(const MarkerData & msg, bool use_flow_style = false)
{
  std::ostringstream out;
  if (use_flow_style) {
    to_flow_style_yaml(msg, out);
  } else {
    to_block_style_yaml(msg, out);
  }
  return out.str();
}

}  // namespace msg

}  // namespace blue_detector_msgs

namespace rosidl_generator_traits
{

[[deprecated("use blue_detector_msgs::msg::to_block_style_yaml() instead")]]
inline void to_yaml(
  const blue_detector_msgs::msg::MarkerData & msg,
  std::ostream & out, size_t indentation = 0)
{
  blue_detector_msgs::msg::to_block_style_yaml(msg, out, indentation);
}

[[deprecated("use blue_detector_msgs::msg::to_yaml() instead")]]
inline std::string to_yaml(const blue_detector_msgs::msg::MarkerData & msg)
{
  return blue_detector_msgs::msg::to_yaml(msg);
}

template<>
inline const char * data_type<blue_detector_msgs::msg::MarkerData>()
{
  return "blue_detector_msgs::msg::MarkerData";
}

template<>
inline const char * name<blue_detector_msgs::msg::MarkerData>()
{
  return "blue_detector_msgs/msg/MarkerData";
}

template<>
struct has_fixed_size<blue_detector_msgs::msg::MarkerData>
  : std::integral_constant<bool, false> {};

template<>
struct has_bounded_size<blue_detector_msgs::msg::MarkerData>
  : std::integral_constant<bool, false> {};

template<>
struct is_message<blue_detector_msgs::msg::MarkerData>
  : std::true_type {};

}  // namespace rosidl_generator_traits

#endif  // BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__TRAITS_HPP_
