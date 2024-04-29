// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from custom_interfaces:msg/EMG.idl
// generated code does not contain a copyright notice
#include "custom_interfaces/msg/detail/emg__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
custom_interfaces__msg__EMG__init(custom_interfaces__msg__EMG * msg)
{
  if (!msg) {
    return false;
  }
  // ch1
  // ch2
  return true;
}

void
custom_interfaces__msg__EMG__fini(custom_interfaces__msg__EMG * msg)
{
  if (!msg) {
    return;
  }
  // ch1
  // ch2
}

bool
custom_interfaces__msg__EMG__are_equal(const custom_interfaces__msg__EMG * lhs, const custom_interfaces__msg__EMG * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // ch1
  if (lhs->ch1 != rhs->ch1) {
    return false;
  }
  // ch2
  if (lhs->ch2 != rhs->ch2) {
    return false;
  }
  return true;
}

bool
custom_interfaces__msg__EMG__copy(
  const custom_interfaces__msg__EMG * input,
  custom_interfaces__msg__EMG * output)
{
  if (!input || !output) {
    return false;
  }
  // ch1
  output->ch1 = input->ch1;
  // ch2
  output->ch2 = input->ch2;
  return true;
}

custom_interfaces__msg__EMG *
custom_interfaces__msg__EMG__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__EMG * msg = (custom_interfaces__msg__EMG *)allocator.allocate(sizeof(custom_interfaces__msg__EMG), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(custom_interfaces__msg__EMG));
  bool success = custom_interfaces__msg__EMG__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
custom_interfaces__msg__EMG__destroy(custom_interfaces__msg__EMG * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    custom_interfaces__msg__EMG__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
custom_interfaces__msg__EMG__Sequence__init(custom_interfaces__msg__EMG__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__EMG * data = NULL;

  if (size) {
    data = (custom_interfaces__msg__EMG *)allocator.zero_allocate(size, sizeof(custom_interfaces__msg__EMG), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = custom_interfaces__msg__EMG__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        custom_interfaces__msg__EMG__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
custom_interfaces__msg__EMG__Sequence__fini(custom_interfaces__msg__EMG__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      custom_interfaces__msg__EMG__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

custom_interfaces__msg__EMG__Sequence *
custom_interfaces__msg__EMG__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  custom_interfaces__msg__EMG__Sequence * array = (custom_interfaces__msg__EMG__Sequence *)allocator.allocate(sizeof(custom_interfaces__msg__EMG__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = custom_interfaces__msg__EMG__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
custom_interfaces__msg__EMG__Sequence__destroy(custom_interfaces__msg__EMG__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    custom_interfaces__msg__EMG__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
custom_interfaces__msg__EMG__Sequence__are_equal(const custom_interfaces__msg__EMG__Sequence * lhs, const custom_interfaces__msg__EMG__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!custom_interfaces__msg__EMG__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
custom_interfaces__msg__EMG__Sequence__copy(
  const custom_interfaces__msg__EMG__Sequence * input,
  custom_interfaces__msg__EMG__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(custom_interfaces__msg__EMG);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    custom_interfaces__msg__EMG * data =
      (custom_interfaces__msg__EMG *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!custom_interfaces__msg__EMG__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          custom_interfaces__msg__EMG__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!custom_interfaces__msg__EMG__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
