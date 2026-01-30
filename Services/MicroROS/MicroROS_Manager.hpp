/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef FINEMOTE_MICROROS_MANAGER_HPP
#define FINEMOTE_MICROROS_MANAGER_HPP

#include "MicroROS_App.hpp"
#include "MicroROSPort.hpp"
#include <rmw_microros/rmw_microros.h>

// 状态机定义
enum class MicroROSState {
    WAITING_AGENT,  // 等待 Agent 上线 (Ping)
    INITIALIZING,   // 正在创建 Node/Pub/Sub
    RUNNING,        // 正常运行 (Spin)
    ERROR_RECOVERY, // 出错，准备清理
    CLEANUP         // 清理资源中
};

template <typename TransportType> // 例如 MicroROSPort<5>
class MicroROSManager {
public:
    static MicroROSManager &GetInstance() {
        static MicroROSManager instance;
        return instance;
    }

    // 注册用户的业务逻辑
    void RegisterApp(MicroROSApp *app) {
        currentApp = app;
    }

    // FreeRTOS 任务主循环调用的入口
    void RunLoop() {
        // 状态机逻辑
        switch (currentState) {
            case MicroROSState::WAITING_AGENT:
                HandleWaitingAgent();
                break;
            case MicroROSState::INITIALIZING:
                HandleInitializing();
                break;
            case MicroROSState::RUNNING:
                HandleRunning();
                break;
            case MicroROSState::ERROR_RECOVERY:
            case MicroROSState::CLEANUP:
                HandleCleanup();
                break;
        }
    }

private:
    MicroROSManager() {
        // 1. 设置分配器 (引用 microros_allocators.c 中的实现)
        allocator = rcl_get_default_allocator();

        // 2. 设置传输层 (使用 TransportType 提供的静态函数)
        rmw_uros_set_custom_transport(
            true,
            nullptr, // args 可以为空，因为 TransportType 是单例
            TransportType::TransportOpen,
            TransportType::TransportClose,
            TransportType::TransportWrite,
            TransportType::TransportRead
        );
    }

    // --- 状态处理函数 ---

    void HandleWaitingAgent() {
        // 尝试 Ping Agent
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            currentState = MicroROSState::INITIALIZING;
        } else {
            // 延时由外部 Task 控制，或者在这里 osDelay
        }
    }

    void HandleInitializing() {
        if (!currentApp) return;

        rcl_ret_t ret;
        // 1. Init Support
        ret = rclc_support_init(&support, 0, nullptr, &allocator);
        if (ret != RCL_RET_OK) { currentState = MicroROSState::CLEANUP; return; }

        // 2. Init Node
        ret = rclc_node_init_default(&node, "FineMote_node", "", &support);
        if (ret != RCL_RET_OK) { currentState = MicroROSState::CLEANUP; return; }

        // 3. Init Executor
        // 假设最大句柄数为 10，可配置
        ret = rclc_executor_init(&executor, &support.context, 10, &allocator);
        if (ret != RCL_RET_OK) { currentState = MicroROSState::CLEANUP; return; }

        // 4. 用户应用初始化
        if (currentApp->OnInit(node, support, executor)) {
            currentState = MicroROSState::RUNNING;
            last_comm_tick = xTaskGetTickCount();
        } else {
            currentState = MicroROSState::CLEANUP;
        }
    }

    void HandleRunning() {
        // 处理一次任务
        rcl_ret_t ret = rclc_executor_spin_some(&executor, RCL_MS_TO_NS(10)); // 10ms timeout

        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            currentState = MicroROSState::ERROR_RECOVERY;
            return;
        }

        if ((xTaskGetTickCount() - last_comm_tick) > pdMS_TO_TICKS(WATCHDOG_TIMEOUT)) {

            rcl_ret_t ping_ret = rmw_uros_ping_agent(100, 1);

            if (ping_ret == RMW_RET_OK) {
                // Agent 还在，只是比较安静。喂狗，继续运行。
                last_comm_tick = xTaskGetTickCount();
            } else {
                // 切换状态到 CLEANUP，这将触发资源销毁和重连流程
                currentState = MicroROSState::ERROR_RECOVERY; // 或者直接 CLEANUP
            }
        }
    }

    void HandleCleanup() {
        if (currentApp) {
            currentApp->OnDestroy(node);
        }

        rclc_executor_fini(&executor);
        rcl_node_fini(&node);
        rclc_support_fini(&support);

        currentState = MicroROSState::WAITING_AGENT;
    }

    // --- 成员变量 ---
    MicroROSApp *currentApp = nullptr;
    MicroROSState currentState = MicroROSState::WAITING_AGENT;

    rcl_allocator_t allocator;
    rclc_support_t support;
    rcl_node_t node;
    rclc_executor_t executor;

    TickType_t last_comm_tick = 0;
    const uint32_t WATCHDOG_TIMEOUT = 2000;
};

#endif //FINEMOTE_MICROROS_MANAGER_HPP