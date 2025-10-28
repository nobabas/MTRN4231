// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interfaces:srv/MoveRequest.idl
// generated code does not contain a copyright notice

#ifndef INTERFACES__SRV__DETAIL__MOVE_REQUEST__STRUCT_H_
#define INTERFACES__SRV__DETAIL__MOVE_REQUEST__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'command'
#include "rosidl_runtime_c/string.h"
// Member 'positions'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in srv/MoveRequest in the package interfaces.
typedef struct interfaces__srv__MoveRequest_Request
{
  rosidl_runtime_c__String command;
  rosidl_runtime_c__double__Sequence positions;
} interfaces__srv__MoveRequest_Request;

// Struct for a sequence of interfaces__srv__MoveRequest_Request.
typedef struct interfaces__srv__MoveRequest_Request__Sequence
{
  interfaces__srv__MoveRequest_Request * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__MoveRequest_Request__Sequence;


// Constants defined in the message

/// Struct defined in srv/MoveRequest in the package interfaces.
typedef struct interfaces__srv__MoveRequest_Response
{
  bool success;
} interfaces__srv__MoveRequest_Response;

// Struct for a sequence of interfaces__srv__MoveRequest_Response.
typedef struct interfaces__srv__MoveRequest_Response__Sequence
{
  interfaces__srv__MoveRequest_Response * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interfaces__srv__MoveRequest_Response__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACES__SRV__DETAIL__MOVE_REQUEST__STRUCT_H_
