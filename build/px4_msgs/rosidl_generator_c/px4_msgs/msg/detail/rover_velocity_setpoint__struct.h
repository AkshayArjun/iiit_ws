// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverVelocitySetpoint.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_SETPOINT__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_SETPOINT__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverVelocitySetpoint in the package px4_msgs.
/**
  * Rover Velocity Setpoint
 */
typedef struct px4_msgs__msg__RoverVelocitySetpoint
{
  /// Time since system start
  uint64_t timestamp;
  /// [@range -inf (Backwards), inf (Forwards)] Speed setpoint
  float speed;
  /// [rad] [@range -pi,pi] [@frame NED] [@invalid: NaN, speed is defined in body x direction] Bearing setpoint
  float bearing;
  /// [rad] [@range -pi, pi] [@frame NED] [@invalid NaN, Defaults to vehicle yaw] Mecanum only: Yaw setpoint
  float yaw;
} px4_msgs__msg__RoverVelocitySetpoint;

// Struct for a sequence of px4_msgs__msg__RoverVelocitySetpoint.
typedef struct px4_msgs__msg__RoverVelocitySetpoint__Sequence
{
  px4_msgs__msg__RoverVelocitySetpoint * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverVelocitySetpoint__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_SETPOINT__STRUCT_H_
