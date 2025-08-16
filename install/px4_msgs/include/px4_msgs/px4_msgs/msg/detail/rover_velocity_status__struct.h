// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from px4_msgs:msg/RoverVelocityStatus.idl
// generated code does not contain a copyright notice

#ifndef PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_
#define PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/RoverVelocityStatus in the package px4_msgs.
/**
  * Rover Velocity Status
 */
typedef struct px4_msgs__msg__RoverVelocityStatus
{
  /// Time since system start
  uint64_t timestamp;
  /// [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Measured speed in body x direction
  float measured_speed_body_x;
  /// [m/s] [@range -inf (Backwards), inf (Forwards)] [@frame Body] Speed setpoint in body x direction that is being tracked (Applied slew rates)
  float adjusted_speed_body_x_setpoint;
  /// [] [@range -1, 1] Integral of the PID for the closed loop controller of the speed in body x direction
  float pid_throttle_body_x_integral;
  /// [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Measured speed in body y direction
  float measured_speed_body_y;
  /// [m/s] [@range -inf (Left), inf (Right)] [@frame Body] [@invalid NaN If not mecanum] Mecanum only: Speed setpoint in body y direction that is being tracked (Applied slew rates)
  float adjusted_speed_body_y_setpoint;
  /// [] [@range -1, 1] Mecanum only: Integral of the PID for the closed loop controller of the speed in body y direction
  float pid_throttle_body_y_integral;
} px4_msgs__msg__RoverVelocityStatus;

// Struct for a sequence of px4_msgs__msg__RoverVelocityStatus.
typedef struct px4_msgs__msg__RoverVelocityStatus__Sequence
{
  px4_msgs__msg__RoverVelocityStatus * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} px4_msgs__msg__RoverVelocityStatus__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // PX4_MSGS__MSG__DETAIL__ROVER_VELOCITY_STATUS__STRUCT_H_
