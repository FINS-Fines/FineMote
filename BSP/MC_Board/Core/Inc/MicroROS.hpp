/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICOROS_HPP
#define FINEMOTE_MICOROS_HPP

#include <cstddef>
#include <cstdint>

#ifdef __cplusplus
extern "C" {
#endif

#include "main.h"
#include "cmsis_os.h"
#include <rcl/rcl.h>
#include <rcl/allocator.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#ifdef __cplusplus
}
#endif

extern "C" {
 // Transport layer functions (assumed to be in main.cpp)
 bool cubemx_transport_open(struct uxrCustomTransport * transport);
 bool cubemx_transport_close(struct uxrCustomTransport * transport);
 size_t cubemx_transport_write(struct uxrCustomTransport* transport, const uint8_t * buf, size_t len, uint8_t * err);
 size_t cubemx_transport_read(struct uxrCustomTransport* transport, uint8_t* buf, size_t len, int timeout, uint8_t* err);

 // Memory allocator functions (defined in microros_allocators.c)
 void* microros_allocate(size_t size, void* state);
 void microros_deallocate(void* pointer, void* state);
 void* microros_reallocate(void* pointer, size_t size, void* state);
 void* microros_zero_allocate(size_t number_of_elements, size_t size_of_element, void* state);
}


template <UART_HandleTypeDef* UartHandlePtr>
class MicroROS {
public:
  static MicroROS &getInstance() {
   static MicroROS instance;
   return instance;
  }


 rcl_allocator_t getAllocator() {
   return _allocator;
  }

private:
 MicroROS() {
  init_transport();
  init_allocator();
 }

 void init_transport() {
  rmw_uros_set_custom_transport(
      true,
      (void*)UartHandlePtr,
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read
  );
 }

 void init_allocator() {
  _allocator = rcutils_get_zero_initialized_allocator();

  _allocator.allocate = microros_allocate;
  _allocator.deallocate = microros_deallocate;
  _allocator.reallocate = microros_reallocate;
  _allocator.zero_allocate = microros_zero_allocate;

  if (!rcutils_set_default_allocator(&_allocator)) {
   // Enter an error state if the allocator cannot be set.
   while (1) {}
  }
 }

 rcl_allocator_t _allocator;
};

#endif //FINEMOTE_MICOROS_HPP