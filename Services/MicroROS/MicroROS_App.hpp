/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_APP_HPP
#define FINEMOTE_MICROROS_APP_HPP

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>

/**
 * @brief micro-ROS 用户应用抽象基类
 * 用户需继承此类并实现具体的 Publisher/Subscriber 逻辑
 */
class MicroROSApp {
public:
    virtual ~MicroROSApp() = default;

    /**
     * @brief 当连接建立，需要创建实体时调用
     * @param node_handle  已初始化的节点句柄
     * @param support      已初始化的支持结构体
     * @param executor     已初始化的执行器
     * @return true 初始化成功, false 初始化失败(将触发重置)
     */
    virtual bool OnInit(rcl_node_t &node_handle, rclc_support_t &support, rclc_executor_t &executor) = 0;

    /**
     * @brief 当连接断开或出错，需要清理资源时调用
     * @param node_handle  节点句柄
     */
    virtual void OnDestroy(rcl_node_t &node_handle) = 0;
};

#endif //FINEMOTE_MICROROS_APP_HPP