/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

#include "main.h"
#include "cmsis_os.h"

// 引入架构头文件
#include "MicroROS/MicroROS_Manager.hpp"
#include "MicroROS/MicroROSPort.hpp"

// 引入具体的 ROS 消息头文件 (Micro-ROS 生成的 C 头文件)
#include <std_msgs/msg/int32.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/header.h>
#include <geometry_msgs/msg/twist.h>
#include <nav_msgs/msg/odometry.h>
// #include  <geometry_msgs/msg/pose_stamped.h>
Timer* Timer::head_ = nullptr;

// =============================================================================
// 1. 系统配置与桥接 (Glue Code)
// =============================================================================

// 定义使用的硬件端口 (例如 UART5)
using MyTransport = MicroROSPort<5>;
// 定义管理器类型
using MyManager = MicroROSManager<MyTransport>;

// 【核心】实现实体注册的桥接函数
// 这个函数被 MicroROSEntity 的构造函数调用
namespace Internal {
    void RegisterEntityToManager(MicroROSEntity* entity) {
        MyManager::GetInstance().RegisterEntity(entity);
    }
}

// =============================================================================
// 2. 消息类型注册 (Type Traits)
// =============================================================================

// 必须为每一个用到的消息类型进行宏定义，否则编译报错
// 格式: DEFINE_MICROROS_MSG_TYPE(C++类型, 包名, msg, 消息名)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Int32, std_msgs, msg, Int32)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Bool, std_msgs, msg, Bool)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Header, std_msgs, msg, Header)

DEFINE_MICROROS_MSG_TYPE(geometry_msgs__msg__Twist, geometry_msgs, msg, Twist)
DEFINE_MICROROS_MSG_TYPE(nav_msgs__msg__Odometry, nav_msgs, msg, Odometry)
// DEFINE_MICROROS_MSG_TYPE(geometry_msgs__msg__PoseStamped, geometry_msgs, msg, PoseStamped) // <--- 新增

// =============================================================================
// 3. 用户业务逻辑 (User Application)
// =============================================================================

// --- 3.1 定义发布者 ---
// 声明后自动注册，无需 Init 代码
Publisher<std_msgs__msg__Int32> pub_heartbeat("heartbeat");
Publisher<std_msgs__msg__Header> pub_sensor("sensor_info");
Publisher<nav_msgs__msg__Odometry> pub_odom("odom");
// Publisher<geometry_msgs__msg__PoseStamped> pub_odom("odom");

// --- 3.2 定义回调函数 ---
void OnLedCommand(const std_msgs__msg__Bool& msg) {
    if (msg.data) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_SET);
    }
}

void OnCmdVel(const geometry_msgs__msg__Twist& msg) {
    // 从 ROS 消息中提取线速度(x, y)和角速度(z)
    // 直接传给桥接函数，写入底盘任务
    // POVChassis_SetTargetSpeed_Safe(msg.linear.x, msg.linear.y, msg.angular.z);
}
// --- 3.3 定义订阅者 ---
// 传入 Topic 名称和回调函数
Subscriber<std_msgs__msg__Bool> sub_led("led_cmd", OnLedCommand);
Subscriber<geometry_msgs__msg__Twist> sub_cmd_vel("cmd_vel", OnCmdVel);

// --- 3.4 定义定时器回调 ---
void OnTimerCallback() {
    static int32_t counter = 0;

    // 业务 1: 发布心跳数字
    auto& msg_hb = pub_heartbeat.load_msg();
    msg_hb.data = counter++;
    pub_heartbeat.publish();

    // 业务 2: 发布带时间戳的消息
    auto& msg_sensor = pub_sensor.load_msg();

    // 填充时间戳 (使用 FreeRTOS Tick 估算)
    TickType_t ticks = xTaskGetTickCount();
    msg_sensor.stamp.sec = ticks / configTICK_RATE_HZ;
    msg_sensor.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ);

    // 填充 Frame ID
    static char frame_id[] = "stm32_frame";
    msg_sensor.frame_id.data = frame_id;
    msg_sensor.frame_id.size = strlen(frame_id);
    msg_sensor.frame_id.capacity = sizeof(frame_id);

    pub_sensor.publish();

    auto& msg_odom = pub_odom.load_msg();

    ticks = xTaskGetTickCount();
    msg_odom.header.stamp.sec = ticks / configTICK_RATE_HZ;
    msg_odom.header.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ);

    static char frame_id_odom[] = "odom";
    msg_odom.header.frame_id.data = frame_id_odom;
    msg_odom.header.frame_id.size = strlen(frame_id_odom);
    msg_odom.header.frame_id.capacity = sizeof(frame_id_odom);

    static char frame_id_base[] = "base_link";
    msg_odom.child_frame_id.data = frame_id_base;
    msg_odom.child_frame_id.size = strlen(frame_id_base);
    msg_odom.child_frame_id.capacity = sizeof(frame_id_base);

    // 3. 从底盘任务获取数据
    float chassis_data[6] = {0}; // [x, y, theta, vx, vy, omega]
    // POVChassis_GetState_Safe(chassis_data);

    // 4. 填充位姿 (Pose)
    msg_odom.pose.pose.position.x = chassis_data[0];
    msg_odom.pose.pose.position.y = chassis_data[1];
    msg_odom.pose.pose.position.z = 0.0;

    // 简单的欧拉角转四元数 (仅 Z 轴旋转)
    // qz = sin(theta/2), qw = cos(theta/2)
    float half_theta = chassis_data[2] * 0.5f;
    msg_odom.pose.pose.orientation.z = sinf(half_theta);
    msg_odom.pose.pose.orientation.w = cosf(half_theta);
    msg_odom.pose.pose.orientation.x = 0.0;
    msg_odom.pose.pose.orientation.y = 0.0;

    // 5. 填充速度 (Twist)
    msg_odom.twist.twist.linear.x = chassis_data[3];
    msg_odom.twist.twist.linear.y = chassis_data[4];
    msg_odom.twist.twist.angular.z = chassis_data[5];

    pub_odom.publish();

    // new
    // auto& msg_odom = pub_odom.load_msg();
    //
    // // 1. 填充 Header
    // ticks = xTaskGetTickCount();
    // msg_odom.header.stamp.sec = ticks / configTICK_RATE_HZ;
    // msg_odom.header.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ);
    //
    // static char frame_id_odom[] = "odom";
    // msg_odom.header.frame_id.data = frame_id_odom;
    // msg_odom.header.frame_id.size = strlen(frame_id_odom);
    // msg_odom.header.frame_id.capacity = sizeof(frame_id_odom);
    //
    // // 2. 从底盘任务获取数据 (模拟数据)
    // float chassis_data[6] = {0}; // [x, y, theta, vx, vy, omega]
    // // POVChassis_GetState_Safe(chassis_data);
    //
    // // 3. 填充位姿 (Pose)
    //
    // msg_odom.pose.position.x = chassis_data[0];
    // msg_odom.pose.position.y = chassis_data[1];
    // msg_odom.pose.position.z = 0.0;
    //
    // // 四元数计算
    // float half_theta = chassis_data[2] * 0.5f;
    // msg_odom.pose.orientation.z = sinf(half_theta);
    // msg_odom.pose.orientation.w = cosf(half_theta);
    // msg_odom.pose.orientation.x = 0.0;
    // msg_odom.pose.orientation.y = 0.0;
    //
    // // 发布
    // pub_odom.publish();
}

// --- 3.5 定义定时器 ---
// 500ms 周期，自动统计句柄需求
Timer timer_500ms(500, OnTimerCallback);


// =============================================================================
// 4. FreeRTOS 任务入口
// =============================================================================

extern "C" void StartMicroROSTask(void *argument) {
    // 此时，所有的 Publisher/Subscriber/Timer 已经通过全局构造函数
    // 注册到了 MyManager 内部的列表中。

    // 进入死循环，移交控制权给管理器
    while (true) {
        // RunLoop 内部处理连接、重连、Spin、看门狗等所有状态机逻辑
        MyManager::GetInstance().RunLoop();

        // 必须有延时，让出 CPU 给空闲任务或其他任务
        osDelay(10);
    }
}