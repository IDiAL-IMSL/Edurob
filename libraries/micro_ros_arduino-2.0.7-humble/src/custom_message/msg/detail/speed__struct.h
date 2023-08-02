// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_message:msg/Speed.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SPEED__STRUCT_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__SPEED__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

// Include directives for member types
// Member 'speeds'
#include "rosidl_runtime_c/primitives_sequence.h"

/// Struct defined in msg/Speed in the package custom_message.
/**
  *    Copyright (c) 2023
  *    Fixposition AG
  *
  * Wheel speed input to Fixposition ROS Driver
 */
typedef struct custom_message__msg__Speed
{
  /// Velocity values in [mm/s] (or [mrad/s]) as integer 32bit
  /// 2 Options:
  /// Option 1: One vehicle speed, of sensor RC
  /// Option 2: One vehicle speed, of sensor RC and the yaw rate of the vehicle
  /// Option 3: Fill in 4 Values of 4 wheels, in the order of FR, FL, RR, RL
  rosidl_runtime_c__int32__Sequence speeds;
} custom_message__msg__Speed;

// Struct for a sequence of custom_message__msg__Speed.
typedef struct custom_message__msg__Speed__Sequence
{
  custom_message__msg__Speed * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_message__msg__Speed__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SPEED__STRUCT_H_
