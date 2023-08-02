// generated from rosidl_generator_c/resource/idl__functions.h.em
// with input from custom_message:msg/Speed.idl
// generated code does not contain a copyright notice

#ifndef CUSTOM_MESSAGE__MSG__DETAIL__SPEED__FUNCTIONS_H_
#define CUSTOM_MESSAGE__MSG__DETAIL__SPEED__FUNCTIONS_H_

#ifdef __cplusplus
extern "C"
{
#endif

#include <stdbool.h>
#include <stdlib.h>

#include "rosidl_runtime_c/visibility_control.h"
#include "custom_message/msg/rosidl_generator_c__visibility_control.h"

#include "custom_message/msg/detail/speed__struct.h"

/// Initialize msg/Speed message.
/**
 * If the init function is called twice for the same message without
 * calling fini inbetween previously allocated memory will be leaked.
 * \param[in,out] msg The previously allocated message pointer.
 * Fields without a default value will not be initialized by this function.
 * You might want to call memset(msg, 0, sizeof(
 * custom_message__msg__Speed
 * )) before or use
 * custom_message__msg__Speed__create()
 * to allocate and initialize the message.
 * \return true if initialization was successful, otherwise false
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__init(custom_message__msg__Speed * msg);

/// Finalize msg/Speed message.
/**
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Speed__fini(custom_message__msg__Speed * msg);

/// Create msg/Speed message.
/**
 * It allocates the memory for the message, sets the memory to zero, and
 * calls
 * custom_message__msg__Speed__init().
 * \return The pointer to the initialized message if successful,
 * otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
custom_message__msg__Speed *
custom_message__msg__Speed__create();

/// Destroy msg/Speed message.
/**
 * It calls
 * custom_message__msg__Speed__fini()
 * and frees the memory of the message.
 * \param[in,out] msg The allocated message pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Speed__destroy(custom_message__msg__Speed * msg);

/// Check for msg/Speed message equality.
/**
 * \param[in] lhs The message on the left hand size of the equality operator.
 * \param[in] rhs The message on the right hand size of the equality operator.
 * \return true if messages are equal, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__are_equal(const custom_message__msg__Speed * lhs, const custom_message__msg__Speed * rhs);

/// Copy a msg/Speed message.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source message pointer.
 * \param[out] output The target message pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer is null
 *   or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__copy(
  const custom_message__msg__Speed * input,
  custom_message__msg__Speed * output);

/// Initialize array of msg/Speed messages.
/**
 * It allocates the memory for the number of elements and calls
 * custom_message__msg__Speed__init()
 * for each element of the array.
 * \param[in,out] array The allocated array pointer.
 * \param[in] size The size / capacity of the array.
 * \return true if initialization was successful, otherwise false
 * If the array pointer is valid and the size is zero it is guaranteed
 # to return true.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__Sequence__init(custom_message__msg__Speed__Sequence * array, size_t size);

/// Finalize array of msg/Speed messages.
/**
 * It calls
 * custom_message__msg__Speed__fini()
 * for each element of the array and frees the memory for the number of
 * elements.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Speed__Sequence__fini(custom_message__msg__Speed__Sequence * array);

/// Create array of msg/Speed messages.
/**
 * It allocates the memory for the array and calls
 * custom_message__msg__Speed__Sequence__init().
 * \param[in] size The size / capacity of the array.
 * \return The pointer to the initialized array if successful, otherwise NULL
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
custom_message__msg__Speed__Sequence *
custom_message__msg__Speed__Sequence__create(size_t size);

/// Destroy array of msg/Speed messages.
/**
 * It calls
 * custom_message__msg__Speed__Sequence__fini()
 * on the array,
 * and frees the memory of the array.
 * \param[in,out] array The initialized array pointer.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
void
custom_message__msg__Speed__Sequence__destroy(custom_message__msg__Speed__Sequence * array);

/// Check for msg/Speed message array equality.
/**
 * \param[in] lhs The message array on the left hand size of the equality operator.
 * \param[in] rhs The message array on the right hand size of the equality operator.
 * \return true if message arrays are equal in size and content, otherwise false.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__Sequence__are_equal(const custom_message__msg__Speed__Sequence * lhs, const custom_message__msg__Speed__Sequence * rhs);

/// Copy an array of msg/Speed messages.
/**
 * This functions performs a deep copy, as opposed to the shallow copy that
 * plain assignment yields.
 *
 * \param[in] input The source array pointer.
 * \param[out] output The target array pointer, which must
 *   have been initialized before calling this function.
 * \return true if successful, or false if either pointer
 *   is null or memory allocation fails.
 */
ROSIDL_GENERATOR_C_PUBLIC_custom_message
bool
custom_message__msg__Speed__Sequence__copy(
  const custom_message__msg__Speed__Sequence * input,
  custom_message__msg__Speed__Sequence * output);

#ifdef __cplusplus
}
#endif

#endif  // CUSTOM_MESSAGE__MSG__DETAIL__SPEED__FUNCTIONS_H_
