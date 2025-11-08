// generated from rosidl_generator_cpp/resource/idl__struct.hpp.em
// with input from interface:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_HPP_
#define INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_HPP_

#include <algorithm>
#include <array>
#include <memory>
#include <string>
#include <vector>

#include "rosidl_runtime_cpp/bounded_vector.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


#ifndef _WIN32
# define DEPRECATED__interface__msg__MarkerData __attribute__((deprecated))
#else
# define DEPRECATED__interface__msg__MarkerData __declspec(deprecated)
#endif

namespace interface
{

namespace msg
{

// message struct
template<class ContainerAllocator>
struct MarkerData_
{
  using Type = MarkerData_<ContainerAllocator>;

  explicit MarkerData_(rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0.0f;
    }
  }

  explicit MarkerData_(const ContainerAllocator & _alloc, rosidl_runtime_cpp::MessageInitialization _init = rosidl_runtime_cpp::MessageInitialization::ALL)
  {
    (void)_alloc;
    if (rosidl_runtime_cpp::MessageInitialization::ALL == _init ||
      rosidl_runtime_cpp::MessageInitialization::ZERO == _init)
    {
      this->id = 0.0f;
    }
  }

  // field types and members
  using _id_type =
    float;
  _id_type id;
  using _pose_type =
    std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>>;
  _pose_type pose;

  // setters for named parameter idiom
  Type & set__id(
    const float & _arg)
  {
    this->id = _arg;
    return *this;
  }
  Type & set__pose(
    const std::vector<float, typename std::allocator_traits<ContainerAllocator>::template rebind_alloc<float>> & _arg)
  {
    this->pose = _arg;
    return *this;
  }

  // constant declarations

  // pointer types
  using RawPtr =
    interface::msg::MarkerData_<ContainerAllocator> *;
  using ConstRawPtr =
    const interface::msg::MarkerData_<ContainerAllocator> *;
  using SharedPtr =
    std::shared_ptr<interface::msg::MarkerData_<ContainerAllocator>>;
  using ConstSharedPtr =
    std::shared_ptr<interface::msg::MarkerData_<ContainerAllocator> const>;

  template<typename Deleter = std::default_delete<
      interface::msg::MarkerData_<ContainerAllocator>>>
  using UniquePtrWithDeleter =
    std::unique_ptr<interface::msg::MarkerData_<ContainerAllocator>, Deleter>;

  using UniquePtr = UniquePtrWithDeleter<>;

  template<typename Deleter = std::default_delete<
      interface::msg::MarkerData_<ContainerAllocator>>>
  using ConstUniquePtrWithDeleter =
    std::unique_ptr<interface::msg::MarkerData_<ContainerAllocator> const, Deleter>;
  using ConstUniquePtr = ConstUniquePtrWithDeleter<>;

  using WeakPtr =
    std::weak_ptr<interface::msg::MarkerData_<ContainerAllocator>>;
  using ConstWeakPtr =
    std::weak_ptr<interface::msg::MarkerData_<ContainerAllocator> const>;

  // pointer types similar to ROS 1, use SharedPtr / ConstSharedPtr instead
  // NOTE: Can't use 'using' here because GNU C++ can't parse attributes properly
  typedef DEPRECATED__interface__msg__MarkerData
    std::shared_ptr<interface::msg::MarkerData_<ContainerAllocator>>
    Ptr;
  typedef DEPRECATED__interface__msg__MarkerData
    std::shared_ptr<interface::msg::MarkerData_<ContainerAllocator> const>
    ConstPtr;

  // comparison operators
  bool operator==(const MarkerData_ & other) const
  {
    if (this->id != other.id) {
      return false;
    }
    if (this->pose != other.pose) {
      return false;
    }
    return true;
  }
  bool operator!=(const MarkerData_ & other) const
  {
    return !this->operator==(other);
  }
};  // struct MarkerData_

// alias to use template instance with default allocator
using MarkerData =
  interface::msg::MarkerData_<std::allocator<void>>;

// constant definitions

}  // namespace msg

}  // namespace interface

#endif  // INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_HPP_
