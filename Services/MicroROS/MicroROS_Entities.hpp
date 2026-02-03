/*******************************************************************************
* Copyright (c) 2026.
* IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
* All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_MICROROS_ENTITIES_HPP
#define FINEMOTE_MICROROS_ENTITIES_HPP

#include <functional>
#include "etl/vector.h"
#include <string.h>

// Micro-ROS C Includes
#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

// =============================================================================
// 1. 类型特征 (Type Traits) & 辅助宏
// =============================================================================

/**
 * @brief 消息类型特征结构体
 * 用户需要通过下方的 DEFINE_MICROROS_MSG_TYPE 宏来特化此结构体，
 * 从而让模板类能够自动获取 ROSIDL 的类型支持句柄。
 */
template<typename T>
struct RosMsgTypeTraits {
    static const rosidl_message_type_support_t* GetTypeSupport() {
        return nullptr; // 默认实现，如果没有特化则报错或返回空
    }
};

/**
 * @brief 宏：关联 C++ 消息类型与 Micro-ROS C 类型支持
 * 使用示例: DEFINE_MICROROS_MSG_TYPE(std_msgs::msg::Header, std_msgs, msg, Header)
 */
#define DEFINE_MICROROS_MSG_TYPE(CppType, PkgName, MsgSub, MsgName) \
    template<> \
    struct RosMsgTypeTraits<CppType> { \
        static const rosidl_message_type_support_t* GetTypeSupport() { \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName); \
        } \
    };

// =============================================================================
// 2. 基础定义
// =============================================================================

enum class QoS {
    BEST_EFFORT,
    RELIABLE
};

/**
 * @brief Micro-ROS 实体抽象基类
 * 所有的 Publisher, Subscriber, Timer 都继承此类，以便 Manager 统一管理
 */
class MicroROSEntity {
public:
    MicroROSEntity(); // 构造函数中将调用 Manager::Register

    virtual ~MicroROSEntity() = default;

    /**
     * @brief 初始化底层句柄 (Node, Pub/Sub/Timer)
     * @param node 已初始化的节点指针
    * @param support 支持结构体指针
     * @return true 成功
     */
    virtual bool Init(rcl_node_t* node, rclc_support_t* support) = 0;

    /**
     * @brief 注册到执行器 (Executor)
     * @param executor 已初始化的执行器指针
     * @return true 成功
     */

    /**
     * @brief 获取此实体需要的 Executor 句柄数量
     * Pub = 0, Sub = 1, Timer = 1
     */
    virtual size_t GetExecutorHandleCount() const {
        return 0;
    }

    /**
     * @brief 注册到执行器 (Executor)
     * @param executor 已初始化的执行器指针
     * @return true 成功
     */
    virtual bool AddToExecutor(rclc_executor_t* executor) {
        // 默认实现为空 (Publisher 不需要加入 Executor)
        return true;
    }

    /**
     * @brief 重置底层句柄 (断连时调用)
     */
    virtual void Reset() = 0;




protected:
    // 辅助：获取 QoS 对应的 rclrmw profile
    const rmw_qos_profile_t* GetQoSProfile(QoS qos) {
        return (qos == QoS::BEST_EFFORT) ? &rmw_qos_profile_sensor_data : &rmw_qos_profile_default;
    }
};

// =============================================================================
// 3. 模板实体实现
// =============================================================================

/**
 * @brief 模板化发布者
 * @tparam MsgType ROS 消息类型 (如 std_msgs::msg::Int32)
 */
template<typename MsgType>
class Publisher : public MicroROSEntity {
public:
    Publisher(const char* topic_name, QoS qos = QoS::BEST_EFFORT)
        : topic_name_(topic_name), qos_(qos), initialized_(false) {}

    // --- 接口方法 ---

    /**
     * @brief 获取内部消息对象的引用，供用户填充数据
     */
    MsgType& load_msg() {
        return msg_;
    }

    /**
     * @brief 发布消息
     */
    bool publish() {
        if (!initialized_) return false;
        rcl_ret_t ret = rcl_publish(&publisher_, &msg_, nullptr);
        return (ret == RCL_RET_OK);
    }

    /**
     * @brief 直接发布外部构建的消息
     */
    bool publish(const MsgType& msg) {
        if (!initialized_) return false;
        rcl_ret_t ret = rcl_publish(&publisher_, &msg, nullptr);
        return (ret == RCL_RET_OK);
    }

    // --- 基类实现 ---

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        (void)support;
        const rosidl_message_type_support_t* type_support =
            RosMsgTypeTraits<MsgType>::GetTypeSupport();

        if (!type_support) return false; // 未特化 Traits

        rcl_ret_t ret;
        if (qos_ == QoS::BEST_EFFORT) {
             ret = rclc_publisher_init_best_effort(
                &publisher_, node, type_support, topic_name_);
        } else {
             ret = rclc_publisher_init_default(
                &publisher_, node, type_support, topic_name_);
        }

        initialized_ = (ret == RCL_RET_OK);
        return initialized_;
    }

    void Reset() override {
        if (initialized_) {
            rcl_publisher_fini(&publisher_, nullptr); // 此时 Node 可能已无效，传 nullptr 安全清理
            initialized_ = false;
        }
    }

private:
    const char* topic_name_;
    QoS qos_;
    rcl_publisher_t publisher_;
    MsgType msg_; // 内部持有的消息实例 (Payload)
    bool initialized_;
};

/**
 * @brief 模板化订阅者
 * @tparam MsgType ROS 消息类型
 */
template<typename MsgType>
class Subscriber : public MicroROSEntity {
public:
    // 回调函数类型定义
    using CallbackFunc = std::function<void(const MsgType&)>;

    Subscriber(const char* topic_name, CallbackFunc callback, QoS qos = QoS::BEST_EFFORT)
        : topic_name_(topic_name), callback_(callback), qos_(qos), initialized_(false) {}

    // --- 基类实现 ---

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        (void)support;
        const rosidl_message_type_support_t* type_support =
            RosMsgTypeTraits<MsgType>::GetTypeSupport();

        if (!type_support) return false;

        rcl_ret_t ret;
        if (qos_ == QoS::BEST_EFFORT) {
            ret = rclc_subscription_init_best_effort(
                &subscription_, node, type_support, topic_name_);
        } else {
            ret = rclc_subscription_init_default(
                &subscription_, node, type_support, topic_name_);
        }

        initialized_ = (ret == RCL_RET_OK);
        return initialized_;
    }

    /**
     * @brief 获取该实体占用的 Executor 句柄数量
     * Publisher = 0
     * Subscriber = 1
     * Timer = 1
     */
    size_t GetExecutorHandleCount() const override {
        return 1;
    }

    /**
     * @brief 注册到执行器 (Executor)
     * @param executor 已初始化的执行器指针
     * @return true 成功
     */
    bool AddToExecutor(rclc_executor_t* executor) override {
        if (!initialized_) return false;

        // 关键：context 传入 this 指针
        rcl_ret_t ret = rclc_executor_add_subscription_with_context(
            executor,
            &subscription_,
            &msg_, // 接收数据存放的缓冲区
            &StaticCallbackTrampoline, // 静态跳板函数
            this, // 上下文
            ON_NEW_DATA
        );
        return (ret == RCL_RET_OK);
    }

    void Reset() override {
        if (initialized_) {
            rcl_subscription_fini(&subscription_, nullptr);
            initialized_ = false;
        }
    }

private:
    // --- 静态回调路由 (Trampoline) ---
    static void StaticCallbackTrampoline(const void * msgin, void * context) {
        // 1. 恢复上下文
        auto* self = static_cast<Subscriber<MsgType>*>(context);
        // 2. 转换消息指针
        const auto* msg = static_cast<const MsgType*>(msgin);

        // 3. 调用用户回调
        if (self && self->callback_) {
            self->callback_(*msg);
        }
    }

    const char* topic_name_;
    CallbackFunc callback_;
    QoS qos_;
    rcl_subscription_t subscription_;
    MsgType msg_; // 接收缓冲区
    bool initialized_;
};

// =============================================================================
// Timer (定时器)
// =============================================================================

class Timer : public MicroROSEntity {
public:
    using TimerCallback = std::function<void()>;

    /**
     * @param period_ms 周期 (毫秒)
     * @param callback 回调函数
     */
    Timer(unsigned int period_ms, TimerCallback callback)
        : period_ms_(period_ms), callback_(callback), initialized_(false), next_(nullptr)
    {
        // 【核心修改】使用链表头插法，自动注册
        // 不需要检查 full()，因为没有上限
        this->next_ = head_;
        head_ = this;
    }

    // --- 基类实现 ---

    // [修改] Timer 需要使用 support 来初始化
    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        (void)node; // Timer init 不需要 node，但需要 support

        rcl_ret_t ret = rclc_timer_init_default(
            &timer_,
            support,
            RCL_MS_TO_NS(period_ms_),
            StaticTimerCallbackTrampoline
        );

        initialized_ = (ret == RCL_RET_OK);
        return initialized_;
    }

    /**
     * @brief 获取该实体占用的 Executor 句柄数量
     * Publisher = 0
     * Subscriber = 1
     * Timer = 1
     */
    size_t GetExecutorHandleCount() const override {
        return 1;
    }

    bool AddToExecutor(rclc_executor_t* executor) override {
        if (!initialized_) return false;
        // Timer 添加时不需要 context (rclc 不支持直接传 ctx 给 timer callback)
        rcl_ret_t ret = rclc_executor_add_timer(executor, &timer_);
        return (ret == RCL_RET_OK);
    }

    void Reset() override {
        if (initialized_) {
            rcl_timer_fini(&timer_);
            initialized_ = false;
        }
    }

private:
    // 静态跳板函数：Micro-ROS C 库调用的入口
    static void StaticTimerCallbackTrampoline(rcl_timer_t * timer, int64_t last_call_time) {
        (void)last_call_time;

        // 遍历链表，找到对应的 C++ 对象
        Timer* current = head_;
        while (current != nullptr) {
            // 通过比较底层 rcl_timer_t 的地址来匹配
            if (&(current->timer_) == timer) {
                if (current->callback_) {
                    current->callback_();
                }
                return;
            }
            current = current->next_; // 找下一个
        }
    }

    unsigned int period_ms_;
    TimerCallback callback_;
    rcl_timer_t timer_;
    bool initialized_;
    Timer* next_;
    static Timer* head_;
};


// =============================================================================
// 4. 全局注册机制实现 (需要在 cpp 中实现 Manager 的调用)
// =============================================================================

// 为了解耦，这里只声明外部函数，实际在 MicroROS_Manager.cpp 中实现
namespace Internal {
    void RegisterEntityToManager(MicroROSEntity* entity);
}

// 构造函数实现：自动注册
inline MicroROSEntity::MicroROSEntity() {
    Internal::RegisterEntityToManager(this);
}

#endif // FINEMOTE_MICROROS_ENTITIES_HPP