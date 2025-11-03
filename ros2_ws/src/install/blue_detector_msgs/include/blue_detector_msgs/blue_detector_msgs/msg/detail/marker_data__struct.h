// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from blue_detector_msgs:msg/MarkerData.idl
// generated code does not contain a copyright notice

#ifndef BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__STRUCT_H_
#define BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__STRUCT_H_

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

/// Struct defined in msg/MarkerData in the package blue_detector_msgs.
/**
  * Marker ID
 */
typedef struct blue_detector_msgs__msg__MarkerData
{
  float id;
  /// Position and orientation [x, y, z, roll, pitch, yaw]
  rosidl_runtime_c__float__Sequence pose;
} blue_detector_msgs__msg__MarkerData;

// Struct for a sequence of blue_detector_msgs__msg__MarkerData.
typedef struct blue_detector_msgs__msg__MarkerData__Sequence
{
  blue_detector_msgs__msg__MarkerData * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} blue_detector_msgs__msg__MarkerData__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // BLUE_DETECTOR_MSGS__MSG__DETAIL__MARKER_DATA__STRUCT_H_
