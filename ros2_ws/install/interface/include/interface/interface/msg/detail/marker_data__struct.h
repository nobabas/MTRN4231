// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from interface:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_H_
#define INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'pose'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/MarkerData in the package interface.
typedef struct interface__msg__MarkerData
{
  float id;
  rosidl_runtime_c__float__Sequence pose;
} interface__msg__MarkerData;

// Struct for a sequence of interface__msg__MarkerData.
typedef struct interface__msg__MarkerData__Sequence
{
  interface__msg__MarkerData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} interface__msg__MarkerData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // INTERFACE__MSG__DETAIL__MARKER_DATA__STRUCT_H_
