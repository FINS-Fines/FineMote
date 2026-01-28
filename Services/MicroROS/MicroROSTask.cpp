/*******************************************************************************
* Copyright (c) 2025.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
*******************************************************************************/
#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
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


// --- 内存监视结构体 (用于打断点查看) ---
typedef struct {
    size_t free_heap;           // 当前可用堆大小
    size_t min_ever_heap;       // 历史最低可用堆
    UBaseType_t stack_watermark; // 任务栈剩余水位线 (单位: 4字节)
    rcl_ret_t last_ret;         // 最后一个 ROS 函数的返回值
    int step;                   // 当前运行到哪一步
} MicroROS_Debug_t;

MicroROS_Debug_t debug_mem;

// 采样宏：更新监视变量
inline void sample_memory_debug(int s, rcl_ret_t r) {
    debug_mem.step = s;
    debug_mem.last_ret = r;
    debug_mem.free_heap = xPortGetFreeHeapSize();
    debug_mem.min_ever_heap = xPortGetMinimumEverFreeHeapSize();
    debug_mem.stack_watermark = uxTaskGetStackHighWaterMark(NULL);
    __NOP(); // <--- 在这一行打断点，程序每次调用采样都会停在这里
}

// --- 应用常量定义 ---
#define STRING_BUFFER_LEN 50
#define DEVICE_ID_PREFIX "STM32"  // 设备前缀
#define AGENT_RECONNECT_TIMEOUT_MS 500  // 重连重试间隔
#define AGENT_PING_TIMEOUT_MS      100  // Ping 超时时间
#define HEARTBEAT_CHECK_MS         2000 // 运行时心跳检测间隔

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
    ERROR_EXECUTOR_ADD_TIMER = 11,
    ERROR_DISCONNECTED = 99
} ErrorCode_t;

volatile ErrorCode_t g_last_error = ERROR_NONE;  // 全局错误码（可在调试器中查看）

// --- 全局通信对象 ---
static rcl_publisher_t ping_publisher;
static rcl_publisher_t pong_publisher;
static rcl_subscription_t ping_subscriber;
static rcl_subscription_t pong_subscriber;
static rcl_timer_t ping_timer;
static rcl_node_t node;
static rclc_support_t support;
static rclc_executor_t executor;
static rcl_allocator_t allocator;
// --- 消息缓冲区 ---
static std_msgs__msg__Header incoming_ping;
static std_msgs__msg__Header outcoming_ping;
static std_msgs__msg__Header incoming_pong;

// --- 应用状态变量 ---
static int device_id;
static int seq_no = 0;
static int pong_count = 0;

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
// micro-ROS 定时器回调：每 2 秒发送一次 Ping
// ============================================================================
void ping_timer_callback(rcl_timer_t * timer, int64_t last_call_time)
{
    RCLC_UNUSED(last_call_time);

    if (timer != NULL) {
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

        // 3. 获取时间戳（继续使用 FreeRTOS Tick，因为 STM32 上通常没有 clock_gettime）
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

    // if (cmp_result != 0)
    // {
        // 不是自己的 Ping，回复 Pong
        ping_recv_count++;

        ret = rcl_publish(&pong_publisher, (const void*)msg, NULL);

        if (ret == RCL_RET_OK)
        {
            pong_sent_count++;
        }
    // }
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
extern "C" void StartMicroROSTask(void *argument) {
    rcl_ret_t ret;

    // 0. 配置传输层 (只需一次)
    rmw_uros_set_custom_transport(
        true,
        (void*)&huart5,
        cubemx_transport_open,
        cubemx_transport_close,
        cubemx_transport_write,
        cubemx_transport_read
    );

    allocator = rcl_get_default_allocator();

    simple_srand(xTaskGetTickCount());
    device_id = simple_rand() % 1000;

    // --- 内存预分配 ---
    static char out_ping_buf[STRING_BUFFER_LEN];
    outcoming_ping.frame_id.data = out_ping_buf;
    outcoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    static char in_ping_buf[STRING_BUFFER_LEN];
    incoming_ping.frame_id.data = in_ping_buf;
    incoming_ping.frame_id.capacity = STRING_BUFFER_LEN;

    static char in_pong_buf[STRING_BUFFER_LEN];
    incoming_pong.frame_id.data = in_pong_buf;
    incoming_pong.frame_id.capacity = STRING_BUFFER_LEN;

    // ==========================================
    // 外层大循环：负责 重连 - 运行 - 清理
    // ==========================================
    while(1) {
        TickType_t last_check_tick = xTaskGetTickCount();

        // --- 阶段 A: 阻塞等待 Agent 连接 ---
        // step = -1 表示正在等待连接
        sample_memory_debug(-1, RCL_RET_OK);

        while (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
            // 延时让出 CPU，避免死锁
            osDelay(AGENT_RECONNECT_TIMEOUT_MS);
            sample_memory_debug(-1, RCL_RET_ERROR);
        }

        // --- 阶段 B: 初始化 micro-ROS 实体 ---
        sample_memory_debug(0, RCL_RET_OK);

        // 1. Support Init
        ret = rclc_support_init(&support, 0, NULL, &allocator);
        sample_memory_debug(1, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_SUPPORT_INIT; goto cleanup; }

        // 2. Node Init
        ret = rclc_node_init_default(&node, "pingpong_node", "", &support);
        sample_memory_debug(2, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_NODE_INIT; goto cleanup; }

        // 3. Publisher Init (Ping)
        ret = rclc_publisher_init_best_effort(&ping_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "ping");
        sample_memory_debug(3, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_PING_PUB_INIT; goto cleanup; }

        // 4. Publisher Init (Pong)
        ret = rclc_publisher_init_best_effort(&pong_publisher, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "pong");
        sample_memory_debug(4, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_PONG_PUB_INIT; goto cleanup; }

        // 5. Subscriber Init (Ping)
        ret = rclc_subscription_init_best_effort(&ping_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "ping");
        sample_memory_debug(5, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_PING_SUB_INIT; goto cleanup; }

        // 6. Subscriber Init (Pong)
        ret = rclc_subscription_init_best_effort(&pong_subscriber, &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header), "pong");
        sample_memory_debug(6, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_PONG_SUB_INIT; goto cleanup; }

        // 7. Timer Init
        ret = rclc_timer_init_default(&ping_timer, &support, RCL_MS_TO_NS(2000), ping_timer_callback);
        sample_memory_debug(7, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_TIMER_CREATE; goto cleanup; }

        // 8. Executor Init
        ret = rclc_executor_init(&executor, &support.context, 3, &allocator);
        sample_memory_debug(8, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_EXECUTOR_INIT; goto cleanup; }

        // 9. Executor Add Subscriptions
        ret = rclc_executor_add_subscription(&executor, &ping_subscriber, &incoming_ping,
            &ping_subscription_callback, ON_NEW_DATA);
        sample_memory_debug(9, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_EXECUTOR_ADD_PING; goto cleanup; }

        ret = rclc_executor_add_subscription(&executor, &pong_subscriber, &incoming_pong,
            &pong_subscription_callback, ON_NEW_DATA);
        sample_memory_debug(10, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_EXECUTOR_ADD_PONG; goto cleanup; }

        // 10. Executor Add Timer
        ret = rclc_executor_add_timer(&executor, &ping_timer);
        sample_memory_debug(11, ret);
        if (ret != RCL_RET_OK) { g_last_error = ERROR_EXECUTOR_ADD_TIMER; goto cleanup; }

        // --- 阶段 C: 业务主循环 (Spin) ---
        // for (int i=0; i < 200; i++) {
        while (1) {
            // 处理任务
            ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10));
            sample_memory_debug(100, ret);

            // 简单延时
            osDelay(10);

            // --- 心跳检测 ---
            // 每隔 HEARTBEAT_CHECK_MS 检查一次 Agent 是否在线
            if ((xTaskGetTickCount() - last_check_tick) > pdMS_TO_TICKS(HEARTBEAT_CHECK_MS)) {
                last_check_tick = xTaskGetTickCount();

                // 如果 Ping 失败，说明断连，跳出循环进入清理流程
                if (rmw_uros_ping_agent(AGENT_PING_TIMEOUT_MS, 1) != RMW_RET_OK) {
                    g_last_error = ERROR_DISCONNECTED;
                    break; // Break inner loop -> Go to cleanup
                }
            }
        }
        // }
        // osDelay(5000);

        // --- 阶段 D: 资源清理 (Cleanup) ---
        cleanup:
        sample_memory_debug(99, RCL_RET_ERROR);

        // 必须按初始化相反的顺序销毁资源，并尽可能忽略返回值(因为是清理阶段)
        // 实际上 rcl 函数会对未初始化的句柄返回错误，这在 cleanup 中是安全的
        rclc_executor_fini(&executor);
        rcl_timer_fini(&ping_timer);
        rcl_subscription_fini(&pong_subscriber, &node);
        rcl_subscription_fini(&ping_subscriber, &node);
        rcl_publisher_fini(&pong_publisher, &node);
        rcl_publisher_fini(&ping_publisher, &node);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        // 稍微延时后重新进入大循环，开始重新寻找 Agent
        osDelay(5000);
    }
}