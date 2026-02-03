/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_MICROROS_MANAGER_HPP
#define FINEMOTE_MICROROS_MANAGER_HPP

#include "FreeRTOS.h"
#include "task.h"

// Micro-ROS includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rmw_microros/rmw_microros.h>

// Project includes
#include "MicroROS_Entities.hpp" // 包含实体定义
#include "etl/vector.h"          // 使用 ETL 容器

// 配置参数
#define MAX_MICROROS_ENTITIES 20 // 系统允许的最大实体数量
#define MICROROS_NODE_NAME "STM32_FineMote_Node"
#define WATCHDOG_TIMEOUT_MS 2000

// 状态机定义
enum class MicroROSState {
    WAITING_AGENT,  // 等待 Agent 上线
    INITIALIZING,   // 正在初始化 Node 和 Entities
    RUNNING,        // 正常运行 (Spinning)
    ERROR_RECOVERY, // 发生错误，准备重置
    DISCONNECTED    // 断开连接 (中间态)
};

/**
 * @brief Micro-ROS 中心管理器
 * 负责管理生命周期、资源分配以及所有发布者/订阅者的注册
 *
 * @tparam TransportType 传输层类 (例如 MicroROSPort<UART_NUM>)
 */
template <typename TransportType>
class MicroROSManager {
public:
    static MicroROSManager &GetInstance() {
        static MicroROSManager instance;
        return instance;
    }

    /**
     * @brief 注册实体 (由 MicroROSEntity 构造函数自动调用)
     * @param entity 实体指针
     */
    void RegisterEntity(MicroROSEntity* entity) {
        if (!entity_list_.full()) {
            entity_list_.push_back(entity);
        } else {
            // 错误处理：实体数量超过 MAX_MICROROS_ENTITIES
            // 可以打印日志或闪灯
        }
    }

    /**
     * @brief 主循环，应在 FreeRTOS 任务中通过 while(1) 调用
     */
    void RunLoop() {
        switch (current_state_) {
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
            case MicroROSState::DISCONNECTED:
                HandleCleanup();
                break;
        }
    }

private:
    MicroROSManager() {
        // 1. 设置内存分配器
        allocator_ = rcl_get_default_allocator();

        // 2. 设置传输层
        // 假设 TransportType 提供了符合 micro-ROS 要求的静态接口
        rmw_uros_set_custom_transport(
            true,
            nullptr, // cookies
            TransportType::TransportOpen,
            TransportType::TransportClose,
            TransportType::TransportWrite,
            TransportType::TransportRead
        );

        current_state_ = MicroROSState::WAITING_AGENT;
    }

    // --- 状态处理逻辑 ---

    void HandleWaitingAgent() {
        // 尝试 Ping Agent，超时时间 100ms，尝试 1 次
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            current_state_ = MicroROSState::INITIALIZING;
        } else {
            // 没连上，稍微延时，避免死循环占用 CPU
            // 注意：外层循环最好也有 osDelay
        }
    }

    void HandleInitializing() {
        rcl_ret_t ret;

        // 1. 初始化 Support
        ret = rclc_support_init(&support_, 0, nullptr, &allocator_);
        if (ret != RCL_RET_OK) { current_state_ = MicroROSState::ERROR_RECOVERY; return; }

        // 2. 初始化 Node
        ret = rclc_node_init_default(&node_, MICROROS_NODE_NAME, "", &support_);
        if (ret != RCL_RET_OK) { current_state_ = MicroROSState::ERROR_RECOVERY; return; }

        // 3. 遍历注册表，初始化所有实体 (Publisher/Subscriber/Timer)
        size_t handles_needed = 0;
        for (auto* entity : entity_list_) {
            if (!entity->Init(&node_, &support_)) {
                // 如果某个实体初始化失败，整个系统回滚
                current_state_ = MicroROSState::ERROR_RECOVERY;
                return;
            }

            handles_needed += entity->GetExecutorHandleCount();
        }

        // 4. 初始化 Executor
        // 句柄数 = 实体数 + 额外保留数 (GUARD_CONDITIONS etc.)
        size_t executor_handles = (handles_needed > 0) ? handles_needed : 1;

        ret = rclc_executor_init(&executor_, &support_.context, executor_handles, &allocator_);
        if (ret != RCL_RET_OK) { current_state_ = MicroROSState::ERROR_RECOVERY; return; }

        // 5. 将实体加入 Executor
        for (auto* entity : entity_list_) {
            if (!entity->AddToExecutor(&executor_)) {
                current_state_ = MicroROSState::ERROR_RECOVERY;
                return;
            }
        }

        // 初始化成功，进入运行态
        current_state_ = MicroROSState::RUNNING;
        last_comm_tick_ = xTaskGetTickCount();
    }

    void HandleRunning() {
        // 执行一次 Spin (处理订阅回调和定时器)
        // timeout 设为 0 或很小的值，非阻塞
        rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(5));

        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            current_state_ = MicroROSState::ERROR_RECOVERY;
            return;
        }

        // 简单的看门狗逻辑：定期 Ping 确保连接存活
        if ((xTaskGetTickCount() - last_comm_tick_) > pdMS_TO_TICKS(WATCHDOG_TIMEOUT_MS)) {
            if (rmw_uros_ping_agent(50, 1) == RMW_RET_OK) {
                last_comm_tick_ = xTaskGetTickCount();
            } else {
                // Ping 失败，认为连接断开
                current_state_ = MicroROSState::ERROR_RECOVERY;
            }
        }
    }

    void HandleCleanup() {
        // 1. 清理所有实体 (释放 Pub/Sub 句柄)
        for (auto* entity : entity_list_) {
            entity->Reset();
        }

        // 2. 清理 Micro-ROS 核心资源
        rclc_executor_fini(&executor_);
        rcl_node_fini(&node_);
        rclc_support_fini(&support_);

        // 3. 回到等待状态
        current_state_ = MicroROSState::WAITING_AGENT;
    }

    // --- 成员变量 ---

    // Micro-ROS 句柄
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;

    // 状态管理
    MicroROSState current_state_;
    TickType_t last_comm_tick_;

    // 实体注册表
    etl::vector<MicroROSEntity*, MAX_MICROROS_ENTITIES> entity_list_;
};

#endif // FINEMOTE_MICROROS_MANAGER_HPP