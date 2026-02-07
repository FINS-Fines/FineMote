/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#include "cmsis_os.h"
#include "ProjectConfig.h"

#include "BSP_MicroROS.hpp"
#include "Bus/MicroROS_Base.hpp"

#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>

ROSAgent* ROSAgent::head_ = nullptr;

DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Int32, std_msgs, msg, Int32)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Bool, std_msgs, msg, Bool)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Header, std_msgs, msg, Header)
DEFINE_MICROROS_MSG_TYPE(geometry_msgs__msg__Twist, geometry_msgs, msg, Twist)
DEFINE_MICROROS_MSG_TYPE(nav_msgs__msg__Odometry, nav_msgs, msg, Odometry)

RosPublisher<std_msgs__msg__Int32> pub_heartbeat("system/heartbeat");
RosPublisher<std_msgs__msg__Header> pub_sensor("system/sensor_info");
RosPublisher<nav_msgs__msg__Odometry> pub_odom("odom");

void OnLedCommand(const std_msgs__msg__Bool& msg) {
#ifdef LED_Pin
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

void OnCmdVel(const geometry_msgs__msg__Twist& msg) {
    // Todo
    // float vx = msg.linear.x;
    // float vy = msg.linear.y;
    // float wz = msg.angular.z;
    // POVChassis_SetTargetSpeed(vx, vy, wz);
}

RosSubscriber<std_msgs__msg__Bool> sub_led("led_cmd", OnLedCommand);
RosSubscriber<geometry_msgs__msg__Twist> sub_cmd_vel("cmd_vel", OnCmdVel);

void OnTimerCallback() {
    static int32_t counter = 0;

    auto& msg_hb = pub_heartbeat.load_msg();
    msg_hb.data = counter++;
    pub_heartbeat.publish();

    auto& msg_sensor = pub_sensor.load_msg();
    TickType_t ticks = xTaskGetTickCount();
    msg_sensor.stamp.sec = ticks / configTICK_RATE_HZ;
    msg_sensor.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ);

    const char frame_id[] = "stm32_link";
    msg_sensor.frame_id.data = (char*)frame_id;
    msg_sensor.frame_id.size = strlen(frame_id);
    msg_sensor.frame_id.capacity = sizeof(frame_id);

    pub_sensor.publish();

    // Todo
    // auto& msg_odom = pub_odom.load_msg();
    // pub_odom.publish();
}

Timer timer_500ms(500, OnTimerCallback);

using ROS_Manger = BSP_MicroROS<5>;

extern "C" void StartMicroROSTask(void *argument) {

    ROS_Manger::GetInstance().Init();

    for(;;) {
        ROS_Manger::GetInstance().Spin();

        osDelay(10);
    }
}