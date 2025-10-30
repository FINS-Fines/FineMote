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

void StartMicroROSTask()
{
	rcl_allocator_t allocator;
	allocator = rcl_get_default_allocator();

	//create init_options
	rclc_support_t support;
	// char* _argv[] = {NULL};

	volatile rcl_ret_t ret;

	// if (ret != RCL_RET_OK)
	// {
	// 	// 初始化失败，进入死循环并快速闪烁LED报错
	do {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
		if (ret != RCL_RET_OK) {
			osDelay(1000); // 延时 1 秒再试
		}
	} while (ret != RCL_RET_OK);
	// }
	// create node
	// rclc_node_init_default(&node, "cubemx_node", "", &support);
	if (ret != RCL_RET_OK) {
		ret = rclc_support_init(&support, 0, NULL, &allocator);
	}

	rcl_publisher_t publisher;
	std_msgs__msg__Int32 msg;


	rcl_node_t node;
	ret = rclc_node_init_default(&node, "cubemx_node", "", &support);

	// if (ret != RCL_RET_OK)
	// {
	// 	// 初始化失败，进入死循环并快速闪烁LED报错
	// 	// 节点创建失败 (闪2次)
	// 	while(1){
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(700);
	// 	}
	// }
	// create publisher
	ret = rclc_publisher_init_default(
	  &publisher,
	  &node,
	  ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Int32),
	  "cube_node");
	// if (ret != RCL_RET_OK)
	// {
	// 	// 发布者创建失败 (闪3次)
	// 	while(1){
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(100);
	// 		HAL_GPIO_TogglePin(LED_PORT, LED_PIN); osDelay(500);
	// 	}
	// }
	msg.data = 0;

	for(;;)
	{
	    ret = rcl_publish(&publisher, &msg, NULL);
		// if (ret != RCL_RET_OK)
		// {
		// 	// 发布失败。在循环中，我们通常不希望系统卡死。
		// 	// 可以在这里做一个短暂的、不那么激进的报错提示，
		// 	// 比如让LED快速闪一下，然后继续尝试。
		// 	// 如果 Agent 断开连接，这里会持续报错。
		// 	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_SET);
		// 	osDelay(10);
		// 	HAL_GPIO_WritePin(LED_PORT, LED_PIN, GPIO_PIN_RESET);
		// }
		msg.data++;
		osDelay(500);
	}
}