// generated from rosidl_generator_cpp/resource/idl__builder.hpp.em
// with input from interfaces:srv/MoveRequest.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_REQUEST__BUILDER_HPP_
#define INTERFACES__SRV__DETAIL__MOVE_REQUEST__BUILDER_HPP_

#include <algorithm>
#include <utility>

#include "interfaces/srv/detail/move_request__struct.hpp"
#include "rosidl_runtime_cpp/message_initialization.hpp"


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveRequest_Request_positions
{
public:
  explicit Init_MoveRequest_Request_positions(::interfaces::srv::MoveRequest_Request & msg)
  : msg_(msg)
  {}
  ::interfaces::srv::MoveRequest_Request positions(::interfaces::srv::MoveRequest_Request::_positions_type arg)
  {
    msg_.positions = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveRequest_Request msg_;
};

class Init_MoveRequest_Request_command
{
public:
  Init_MoveRequest_Request_command()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  Init_MoveRequest_Request_positions command(::interfaces::srv::MoveRequest_Request::_command_type arg)
  {
    msg_.command = std::move(arg);
    return Init_MoveRequest_Request_positions(msg_);
  }

private:
  ::interfaces::srv::MoveRequest_Request msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveRequest_Request>()
{
  return interfaces::srv::builder::Init_MoveRequest_Request_command();
}

}  // namespace interfaces


namespace interfaces
{

namespace srv
{

namespace builder
{

class Init_MoveRequest_Response_success
{
public:
  Init_MoveRequest_Response_success()
  : msg_(::rosidl_runtime_cpp::MessageInitialization::SKIP)
  {}
  ::interfaces::srv::MoveRequest_Response success(::interfaces::srv::MoveRequest_Response::_success_type arg)
  {
    msg_.success = std::move(arg);
    return std::move(msg_);
  }

private:
  ::interfaces::srv::MoveRequest_Response msg_;
};

}  // namespace builder

}  // namespace srv

template<typename MessageType>
auto build();

template<>
inline
auto build<::interfaces::srv::MoveRequest_Response>()
{
  return interfaces::srv::builder::Init_MoveRequest_Response_success();
}

}  // namespace interfaces

#endif  // INTERFACES__SRV__DETAIL__MOVE_REQUEST__BUILDER_HPP_
