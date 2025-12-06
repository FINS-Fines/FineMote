/*******************************************************************************
* Copyright (c) 2025.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"  // FreeRTOS 软件定时器
#include "main.h"
#include "cmsis_os.h"

#include <string.h>
#include <stdint.h>
#include "usart.h"

// --- micro-ROS includes ---
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// --- ROS Message includes ---
#include <std_msgs/msg/header.h>

#include <stdio.h>

#include "MicroROS.hpp"

// --- 应用常量定义 ---
#define STRING_BUFFER_LEN 50
#define DEVICE_ID_PREFIX "STM32"  // 设备前缀

// --- 错误码定义（用于调试） ---
typedef enum {
    ERROR_NONE = 0,
    ERROR_SUPPORT_INIT = 1,
    ERROR_NODE_INIT = 2,
    ERROR_PING_PUB_INIT = 3,
    ERROR_PONG_PUB_INIT = 4,
    ERROR_PING_SUB_INIT = 5,
    ERROR_PONG_SUB_INIT = 6,
    ERROR_EXECUTOR_INIT = 7,
    ERROR_EXECUTOR_ADD_PING = 8,
    ERROR_EXECUTOR_ADD_PONG = 9,
    ERROR_TIMER_CREATE = 10,
    ERROR_TIMER_START = 11
} ErrorCode_t;

volatile ErrorCode_t g_last_error = ERROR_NONE;  // 全局错误码（可在调试器中查看）

// --- 全局通信对象 ---
rcl_publisher_t ping_publisher;
rcl_publisher_t pong_publisher;
rcl_subscription_t ping_subscriber;
rcl_subscription_t pong_subscriber;

// --- 消息缓冲区 ---
std_msgs__msg__Header incoming_ping;
std_msgs__msg__Header outcoming_ping;
std_msgs__msg__Header incoming_pong;

// --- 应用状态变量 ---
uint32_t device_id;       // 设备唯一标识（启动时随机生成）
uint32_t seq_no;          // 消息序列号（每次发送递增）
uint32_t pong_count;      // 当前 ping 收到的 pong 数量

// --- 调试计数器 ---
volatile uint32_t ping_sent_count = 0;
volatile uint32_t ping_recv_count = 0;
volatile uint32_t pong_sent_count = 0;
volatile uint32_t pong_recv_count = 0;
volatile uint32_t last_rtt_ms = 0;

// --- FreeRTOS 定时器句柄 ---
TimerHandle_t xPingTimer = NULL;

// --- 简易随机数生成 ---
static uint32_t seed = 0;

uint32_t simple_rand(void)
{
    seed = seed * 1103515245 + 12345;
    return (seed / 65536) % 32768;
}

void simple_srand(uint32_t s)
{
    seed = s;
}

// ============================================================================
// FreeRTOS 定时器回调：每 2 秒发送一次 Ping
// ============================================================================
void vPingTimerCallback(TimerHandle_t xTimer)
{
    volatile rcl_ret_t ret;
    TickType_t ticks;
    const TickType_t ticks_per_second = configTICK_RATE_HZ;

    // 1. 生成新的序列号
    seq_no++;

    // 2. 构造唯一消息 ID：格式 "STM32_<device_id>_<seq_no>"
    snprintf(outcoming_ping.frame_id.data, STRING_BUFFER_LEN,
             "%s_%lu_%lu", DEVICE_ID_PREFIX,
             (unsigned long)device_id, (unsigned long)seq_no);
    outcoming_ping.frame_id.size = strlen(outcoming_ping.frame_id.data);

    // 3. 获取时间戳（使用 FreeRTOS Tick）
    ticks = xTaskGetTickCount();
    outcoming_ping.stamp.sec = ticks / ticks_per_second;
    outcoming_ping.stamp.nanosec = (ticks % ticks_per_second) * (1000000000UL / ticks_per_second);

    // 4. 重置 pong 计数器
    pong_count = 0;

    // 5. 发布 Ping 消息
    ret = rcl_publish(&ping_publisher, (const void*)&outcoming_ping, NULL);

    // 6. 更新统计（用于调试）
    if (ret == RCL_RET_OK)
    {
        ping_sent_count++;
    }
}

// ============================================================================
// Ping 订阅回调：收到其他节点的 Ping，回复 Pong
// ============================================================================
void ping_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
    volatile rcl_ret_t ret;
    int cmp_result;

    // 检查是否是自己发送的（通过比较 frame_id）
    cmp_result = strcmp(outcoming_ping.frame_id.data, msg->frame_id.data);

    if (cmp_result != 0)
    {
        // 不是自己的 Ping，回复 Pong
        ping_recv_count++;

        ret = rcl_publish(&pong_publisher, (const void*)msg, NULL);

        if (ret == RCL_RET_OK)
        {
            pong_sent_count++;
        }
    }
}

// ============================================================================
// Pong 订阅回调：收到对自己 Ping 的回复
// ============================================================================
void pong_subscription_callback(const void * msgin)
{
    const std_msgs__msg__Header * msg = (const std_msgs__msg__Header *)msgin;
    int cmp_result;
    TickType_t current_ticks;
    TickType_t sent_ticks;
    TickType_t rtt_ticks;

    // 检查是否是对当前 Ping 的回复
    cmp_result = strcmp(outcoming_ping.frame_id.data, msg->frame_id.data);

    if (cmp_result == 0)
    {
        pong_count++;
        pong_recv_count++;

        // 计算往返时间（RTT）
        current_ticks = xTaskGetTickCount();
        sent_ticks = outcoming_ping.stamp.sec * configTICK_RATE_HZ;
        sent_ticks += (outcoming_ping.stamp.nanosec / (1000000000UL / configTICK_RATE_HZ));
        rtt_ticks = current_ticks - sent_ticks;
        last_rtt_ms = (rtt_ticks * 1000) / configTICK_RATE_HZ;
    }
}

// ============================================================================
// micro-ROS 主任务
// ============================================================================
extern "C" void StartMicroROSTask(void *argument)
{
    volatile rcl_ret_t ret;
    uint32_t uid_seed;

    // ========================================================================
    // 1. 初始化随机数种子（使用栈地址）
    // ========================================================================
    uid_seed = (uint32_t)&ret;
    simple_srand(uid_seed);
    device_id = simple_rand();
    seq_no = 0;

    // ========================================================================
    // 2. 获取 RCL 分配器
    // ========================================================================
    rcl_allocator_t allocator;
    allocator = MicroROS<&huart5>::getInstance().getAllocator();

    // ========================================================================
    // 3. 初始化 RCL 支持结构（带重试机制）
    // ========================================================================
    rclc_support_t support;

    do
    {
        ret = rclc_support_init(&support, 0, NULL, &allocator);

        if (ret != RCL_RET_OK)
        {
            g_last_error = ERROR_SUPPORT_INIT;
            osDelay(300);
        }
    }
    while (ret != RCL_RET_OK);

    g_last_error = ERROR_NONE;

    // ========================================================================
    // 4. 初始化节点
    // ========================================================================
    rcl_node_t node;
    ret = rclc_node_init_default(&node, "pingpong_stm32_node", "", &support);

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_NODE_INIT;

        while(1)
        {
            osDelay(1000);
        }
    }

    // ========================================================================
    // 5. 初始化发布者
    // ========================================================================
    // Ping 发布者（RELIABLE QoS）
    ret = rclc_publisher_init_default(&ping_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping");

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_PING_PUB_INIT;

        while(1)
        {
            osDelay(1000);
        }
    }

    // Pong 发布者（BEST_EFFORT QoS）
    ret = rclc_publisher_init_best_effort(&pong_publisher, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong");

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_PONG_PUB_INIT;

        while(1)
        {
            osDelay(1000);
        }
    }

    // ========================================================================
    // 6. 初始化订阅者
    // ========================================================================
    // Ping 订阅者（BEST_EFFORT QoS）
    ret = rclc_subscription_init_best_effort(&ping_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/ping");

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_PING_SUB_INIT;

        while(1)
        {
            osDelay(1000);
        }
    }

    // Pong 订阅者（BEST_EFFORT QoS）
    ret = rclc_subscription_init_best_effort(&pong_subscriber, &node,
        ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "/microROS/pong");

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_PONG_SUB_INIT;

        while(1)
        {
            osDelay(1000);
        }
    }

    // ========================================================================
    // 7. 初始化执行器（管理 2 个订阅者）
    // ========================================================================

    // 在 rclc_executor_init 调用前添加：

    // 1. 检查可用堆内存
    volatile size_t heap_before = xPortGetFreeHeapSize();
    // 期望值：应该 > 10KB

    // 2. 检查 context 有效性
    volatile int is_valid = rcl_context_is_valid(&support.context);
    // 期望值：应该返回 1 (true)

    // 3. 检查 allocator
    volatile void* alloc_fn = (void*)allocator.allocate;
    // 期望值：不应该是 NULL 或 0xA5A5A5A5

    rclc_executor_t executor;
    ret = rclc_executor_init(&executor, &support.context, 3, &allocator);

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_EXECUTOR_INIT;

        while(1)
        {
            ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
            osDelay(1000);
        }
    }

    // 添加 Ping 订阅者到执行器
    ret = rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
        &ping_subscription_callback, ON_NEW_DATA);

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_EXECUTOR_ADD_PING;

        while(1)
        {
            osDelay(1000);
        }
    }

    // 添加 Pong 订阅者到执行器
    ret = rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
        &pong_subscription_callback, ON_NEW_DATA);

    if (ret != RCL_RET_OK)
    {
        g_last_error = ERROR_EXECUTOR_ADD_PONG;

        while(1)
        {
            osDelay(1000);
        }
    }

    // ========================================================================
    // 8. 分配消息缓冲区（使用 static 避免栈溢出）
    // ========================================================================
    static char outcoming_ping_buffer[STRING_BUFFER_LEN];
    outcoming_ping.frame_id.data = outcoming_ping_buffer;
    outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;
    outcoming_ping.frame_id.size = 0;

    static char incoming_ping_buffer[STRING_BUFFER_LEN];
    incoming_ping.frame_id.data = incoming_ping_buffer;
    incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    static char incoming_pong_buffer[STRING_BUFFER_LEN];
    incoming_pong.frame_id.data = incoming_pong_buffer;
    incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

    // ========================================================================
    // 9. 创建 FreeRTOS 软件定时器（2 秒周期）
    // ========================================================================
    xPingTimer = xTimerCreate(
        "PingTimer",
        pdMS_TO_TICKS(2000),
        pdTRUE,
        (void *)0,
        vPingTimerCallback
    );

    if (xPingTimer == NULL)
    {
    g_last_error = ERROR_TIMER_CREATE;

        while(1)
        {
            osDelay(1000);
        }
    }

    // 启动定时器
    ret = xTimerStart(xPingTimer, 0);

    if (ret != pdPASS)
    {
        g_last_error = ERROR_TIMER_START;

        while(1)
        {
            osDelay(1000);
        }
    }

    // ========================================================================
    // 10. 主循环：持续处理订阅消息
    // ========================================================================
    while (1)
    {
        // 处理订阅消息（超时 10ms）
        ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));

        // 让出 CPU，避免占用过高
        osDelay(10);
    }

    // ========================================================================
    // 11. 清理资源（实际不会执行到）
    // ========================================================================
    xTimerStop(xPingTimer, 0);
    xTimerDelete(xPingTimer, 0);

    rcl_publisher_fini(&ping_publisher, &node);
    rcl_publisher_fini(&pong_publisher, &node);
    rcl_subscription_fini(&ping_subscriber, &node);
    rcl_subscription_fini(&pong_subscriber, &node);
    rcl_node_fini(&node);
    rclc_support_fini(&support);
}
