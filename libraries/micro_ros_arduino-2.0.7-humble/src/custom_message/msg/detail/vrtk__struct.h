// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/VRTK.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__VRTK__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__VRTK__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'header'
#include "std_msgs/msg/detail/header__struct.h"
// Member 'pose_frame'
// Member 'kin_frame'
// Member 'version'
#include "rosidl_runtime_c/string.h"
// Member 'pose'
#include "geometry_msgs/msg/detail/pose_with_covariance__struct.h"
// Member 'velocity'
#include "geometry_msgs/msg/detail/twist_with_covariance__struct.h"
// Member 'acceleration'
#include "geometry_msgs/msg/detail/vector3__struct.h"

/// Struct defined in msg/VRTK in the package custom_message.
/**
  *    Copyright (c) 2023
  *    Fixposition AG
  *
  * Fixposition VRTK Message
 */
typedef struct custom_message__msg__VRTK
{
  std_msgs__msg__Header header;
  /// frame of the pose values [pose, quaternion]
  rosidl_runtime_c__String pose_frame;
  /// frame of the kinematic values [linear/angular velocity, acceleration]
  rosidl_runtime_c__String kin_frame;
  /// position, orientation
  geometry_msgs__msg__PoseWithCovariance pose;
  /// linear, angular
  geometry_msgs__msg__TwistWithCovariance velocity;
  /// linear acceleration
  geometry_msgs__msg__Vector3 acceleration;
  /// field for the fusion status
  int16_t fusion_status;
  /// field for the IMU bias status
  int16_t imu_bias_status;
  /// field for the gnss1 status
  int16_t gnss1_status;
  /// field for the gnss2 status
  int16_t gnss2_status;
  /// field for the wheelspeed status
  int16_t wheelspeed_status;
  /// Fixposition software version
  rosidl_runtime_c__String version;
} custom_message__msg__VRTK;

// Struct for a sequence of custom_message__msg__VRTK.
typedef struct custom_message__msg__VRTK__Sequence
{
  custom_message__msg__VRTK * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__VRTK__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__VRTK__STRUCT_H_
