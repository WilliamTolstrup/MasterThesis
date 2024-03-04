// generated from rosidl_generator_c/resource/idl__struct.h.em
// with input from custom_interfaces:msg/EMG.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_H_
#define CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>


// Constants defined in the message

/// Struct defined in msg/EMG in the package custom_interfaces.
/**
  * two variables for the two channels
 */
typedef struct custom_interfaces__msg__EMG
{
  float ch1;
  float ch2;
} custom_interfaces__msg__EMG;

// Struct for a sequence of custom_interfaces__msg__EMG.
typedef struct custom_interfaces__msg__EMG__Sequence
{
  custom_interfaces__msg__EMG * data;
  /// The number of valid items in data
  size_t size;
  /// The number of allocated items in data
  size_t capacity;
} custom_interfaces__msg__EMG__Sequence;

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_INTERFACES__MSG__DETAIL__EMG__STRUCT_H_
