/*
 * TestTask.cpp
 *
 *  Created on: Jul 9, 2025
 *      Author: pedro
 */

#include <TestTask.h>
#include "System.h"
#include "spi.h"
#include "tim.h"
#include "usbd_cdc_if.h"


// micro-ROS / RCLC
extern "C" {
#include "rmw_microros/rmw_microros.h"
#include "rcl/rcl.h"
#include "rclc/rclc.h"
#include "rclc/executor.h"
#include "std_msgs/msg/string.h"
}

extern "C" {
#include "uxr/client/profile/transport/custom/custom_transport.h"

// Provided by cdc_transport.c (or your chosen transport .c)
bool   cubemx_transport_open(struct uxrCustomTransport * transport);
bool   cubemx_transport_close(struct uxrCustomTransport * transport);
size_t cubemx_transport_write(struct uxrCustomTransport* transport,
                              const uint8_t* buf, size_t len, uint8_t* err);
size_t cubemx_transport_read(struct uxrCustomTransport* transport,
                             uint8_t* buf, size_t len, int timeout, uint8_t* err);


// syscalls_time.c
#include <sys/time.h>
#include "cmsis_os2.h"

int _gettimeofday(struct timeval *tv, void *tzvp)
{
  (void)tzvp;
  // Convert RTOS ticks to seconds/usec
  uint32_t ticks = osKernelGetTickCount();
  uint32_t hz    = osKernelGetTickFreq();   // ticks per second

  tv->tv_sec  = ticks / hz;
  tv->tv_usec = (int)((ticks % hz) * (1000000u / hz));
  return 0;
}
}

static inline void uros_bind_transport_cdc(void)
{
  rmw_uros_set_custom_transport(
      true, NULL,                    // CDC uses no HW handle
      cubemx_transport_open,
      cubemx_transport_close,
      cubemx_transport_write,
      cubemx_transport_read);
}

TestTask::TestTask() : Thread("TestingTask", (osPriority)osPriorityNormal5, (uint32_t) 4096) {}

static rcl_allocator_t      g_alloc;
static rclc_support_t       g_support;
static rcl_node_t           g_node;
static rcl_publisher_t      g_pub;
static rcl_timer_t          g_timer;
static rclc_executor_t      g_exec;
static std_msgs__msg__String g_msg;

static void timer_cb(rcl_timer_t * timer, int64_t)
{
  (void)timer;
  static const char* text = "Hello from STM32H7";
  // Reuse preallocated message buffer
  size_t n = strlen(text);
  if (n >= g_msg.data.capacity) n = g_msg.data.capacity - 1;
  memcpy(g_msg.data.data, text, n);
  g_msg.data.size = n;
  g_msg.data.data[n] = '\0';
  (void)rcl_publish(&g_pub, &g_msg, NULL);
}

void TestTask::init()
{
  // 1) Bind transport (USB-CDC)
  uros_bind_transport_cdc();

  // 2) micro-ROS init
  g_alloc = rcl_get_default_allocator();

  // Support
  rclc_support_init(&g_support, 0, NULL, &g_alloc);

  // Node
  rclc_node_init_default(&g_node, "stm32g4_node", "", &g_support);

  // Publisher
  rclc_publisher_init_default(
      &g_pub, &g_node,
      ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, String),
      "chatter");

  // Message preallocation (avoid heap on publish)
  // Reserve 64 bytes; adjust if you need longer strings
  g_msg.data.data = (char*)g_alloc.allocate(64, g_alloc.state);
  g_msg.data.capacity = 64;
  g_msg.data.size = 0;

  // Timer @ 1 Hz
  rclc_timer_init_default(&g_timer, &g_support, RCL_MS_TO_NS(1000), timer_cb);

  // Executor with 1 handle (the timer)
  rclc_executor_init(&g_exec, &g_support.context, 1, &g_alloc);
  rclc_executor_add_timer(&g_exec, &g_timer);
}

void TestTask::loop()
{
  // Spin micro-ROS executor; keep the task alive
  rclc_executor_spin_some(&g_exec, RCL_MS_TO_NS(5));
  osDelay(1);
}







