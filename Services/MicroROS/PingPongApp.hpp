/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_PINGPONGAPP_HPP
#define FINEMOTE_PINGPONGAPP_HPP

#include "FreeRTOS.h"
#include "task.h"

#include "MicroROS_App.hpp"
#include <std_msgs/msg/header.h>
#include <stdio.h>
#include <string.h>

#define STRING_BUFFER_LEN 64
#define DEVICE_ID "STM32_FineMote"

class PingPongApp : public MicroROSApp {
public:
    PingPongApp() = default;

    // --- 必须实现的接口 ---

    bool OnInit(rcl_node_t &node, rclc_support_t &support, rclc_executor_t &executor) override {
        rcl_ret_t ret;

        instance_ptr = this;

        // 1. 初始化 Publisher (Ping)
        ret = rclc_publisher_init_best_effort(
            &ping_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
            "ping"
        );
        if (ret != RCL_RET_OK) return false;

        // 2. 初始化 Publisher (Pong)
        ret = rclc_publisher_init_best_effort(
            &pong_pub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
            "pong"
        );
        if (ret != RCL_RET_OK) return false;

        // 3. 初始化 Subscriber (Ping)
        ret = rclc_subscription_init_best_effort(
            &ping_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
            "ping"
        );
        if (ret != RCL_RET_OK) return false;

        // 4. 初始化 Subscriber (Pong)
        ret = rclc_subscription_init_best_effort(
            &pong_sub,
            &node,
            ROSIDL_GET_MSG_TYPE_SUPPORT(std_msgs, msg, Header),
            "pong"
        );
        if (ret != RCL_RET_OK) return false;

        // 5. 初始化 Timer (2Hz, 500ms)
        // 注意：context 设置为 this，以便在静态回调中访问成员
        ret = rclc_timer_init_default(
            &ping_timer,
            &support,
            RCL_MS_TO_NS(500),
            PingTimerCallbackStatic
        );

        // 6. 添加到 Executor
        // 注意订阅回调也需要 context，但 rclc_executor_add_subscription
        // 通常将 msg 指针传给回调。我们需要在回调里处理。

        // Add Timer
        ret = rclc_executor_add_timer(&executor, &ping_timer);
        if (ret != RCL_RET_OK) return false;

        // Add Subs
        // 注意：context 参数在 rclc 中通常用于传递消息内存，而不是类实例
        // 这里我们需要预分配消息内存
        ret = rclc_executor_add_subscription_with_context(
            &executor, &ping_sub, &sub_ping_msg, &PingSubCallbackStatic, this, ON_NEW_DATA);
        if (ret != RCL_RET_OK) return false;

        ret = rclc_executor_add_subscription_with_context(
            &executor, &pong_sub, &sub_pong_msg, &PongSubCallbackStatic, this, ON_NEW_DATA);
        if (ret != RCL_RET_OK) return false;

        // 初始化消息内存
        InitMessages();

        return true;
    }

    void OnDestroy(rcl_node_t &node) override {
        (void)rcl_publisher_fini(&ping_pub, &node);
        (void)rcl_publisher_fini(&pong_pub, &node);
        (void)rcl_subscription_fini(&ping_sub, &node);
        (void)rcl_subscription_fini(&pong_sub, &node);
        (void)rcl_timer_fini(&ping_timer);

        instance_ptr = nullptr;
        // 释放消息内存(如果使用了动态分配)
        // 这里使用的是静态Buffer指向，不需要 free
    }

private:
    // --- 静态回调包装器 (Trampolines) ---
    static PingPongApp* instance_ptr;

    static void PingTimerCallbackStatic(rcl_timer_t *timer, int64_t last_call_time) {
        if (instance_ptr) instance_ptr->PublishPing();
    }

    static void PingSubCallbackStatic(const void *msgin, void *context) {
        auto *self = static_cast<PingPongApp *>(context);
        const auto *msg = static_cast<const std_msgs__msg__Header *>(msgin);
        if (self && msg) self->HandlePing(msg);
    }

    static void PongSubCallbackStatic(const void *msgin, void *context) {
        auto *self = static_cast<PingPongApp *>(context);
        const auto *msg = static_cast<const std_msgs__msg__Header *>(msgin);
        if (self && msg) self->HandlePong(msg);
    }

    // --- 实际业务逻辑 ---

    void InitMessages() {
        // 设置 Header 的 frame_id 指针和容量
        pub_msg.frame_id.data = pub_frame_id_buffer;
        pub_msg.frame_id.capacity = STRING_BUFFER_LEN;
        pub_msg.frame_id.size = 0;

        sub_ping_msg.frame_id.data = sub_ping_frame_id_buffer;
        sub_ping_msg.frame_id.capacity = STRING_BUFFER_LEN;

        sub_pong_msg.frame_id.data = sub_pong_frame_id_buffer;
        sub_pong_msg.frame_id.capacity = STRING_BUFFER_LEN;
    }

    void PublishPing() {
        seq_no++;

        // 构造 frame_id: "STM32_FineMote_<seq>"
        snprintf(pub_msg.frame_id.data, STRING_BUFFER_LEN, "%s_%d", DEVICE_ID, seq_no);
        pub_msg.frame_id.size = strlen(pub_msg.frame_id.data);

        // 更新时间戳
        TickType_t ticks = xTaskGetTickCount();
        pub_msg.stamp.sec = ticks / configTICK_RATE_HZ;
        pub_msg.stamp.nanosec = (ticks % configTICK_RATE_HZ) * (1000000000UL / configTICK_RATE_HZ);

        rcl_publish(&ping_pub, &pub_msg, nullptr);
    }

    void HandlePing(const std_msgs__msg__Header *msg) {
        // 收到 Ping，回复 Pong (内容原样发回，或者修改一下)
        // 简单起见，我们把收到的 Ping 转发给 Pong Topic
        rcl_publish(&pong_pub, msg, nullptr);
    }

    void HandlePong(const std_msgs__msg__Header *msg) {
        // 收到 Pong，检查是不是自己发的
        // 实际逻辑可以比较 timestamp 或者 frame_id
        if (strncmp(msg->frame_id.data, DEVICE_ID, strlen(DEVICE_ID)) == 0) {
            // 是自己的回声，计算 RTT 等
            // printf("RTT OK\n");
        }
    }

    // --- 成员变量 ---
    rcl_publisher_t ping_pub;
    rcl_publisher_t pong_pub;
    rcl_subscription_t ping_sub;
    rcl_subscription_t pong_sub;
    rcl_timer_t ping_timer;

    // 消息对象
    std_msgs__msg__Header pub_msg;
    std_msgs__msg__Header sub_ping_msg;
    std_msgs__msg__Header sub_pong_msg;

    // 字符串缓冲区 (避免 malloc)
    char pub_frame_id_buffer[STRING_BUFFER_LEN];
    char sub_ping_frame_id_buffer[STRING_BUFFER_LEN];
    char sub_pong_frame_id_buffer[STRING_BUFFER_LEN];

    int seq_no = 0;
};
PingPongApp* PingPongApp::instance_ptr = nullptr;

#endif //FINEMOTE_PINGPONGAPP_HPP