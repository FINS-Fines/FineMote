/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

#include <stdbool.h>
#include <string.h>
#include <stdio.h>
#include "cmsis_os2.h"
#include "usart.h" // 确保包含了 huart5 的头文件

// --- micro-ROS includes ---

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// --- ROS Message includes ---
#include <std_msgs/msg/header.h>
#include <std_msgs/msg/int32.h>

#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

#include "MicroROS.hpp"

// extern MicroROS<&huart5> micro_ros_handler;

// extern "C" void StartMicroROSTask(void *argument);

extern "C" {

void StartMicroROSTask(void *argument)
{
	rcl_allocator_t allocator;
	allocator = MicroROS<&huart5>::getInstance().getAllocator();

	rclc_support_t support;
	volatile rcl_ret_t ret;

	do {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
		if (ret != RCL_RET_OK) {
			osDelay(1000);
		}
	} while (ret != RCL_RET_OK);

	rcl_node_t node;
	ret = rclc_node_init_default(&node, "cubemx_node", "", &support);
	if (ret != RCL_RET_OK) {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
	}

	rcl_publisher_t publisher;
	std_msgs__msg__Int32 msg;

	ret = rclc_publisher_init_default(
	  &publisher,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	  "cube_node");

	msg.data = 0;

	for(;;)
	{
		ret = rcl_publish(&publisher, &msg, NULL);
		msg.data++;
		osDelay(500);
	}
}
}