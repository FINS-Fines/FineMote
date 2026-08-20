/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_AGENT_HPP
#define FINEMOTE_MICROROS_AGENT_HPP

#include <functional>
#include <string>
#include <type_traits>
#include <utility>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>

#include "MicroROS/MicroROS_MessageTypes.hpp"

#ifndef MICROROS_NODE_NAME
    #define MICROROS_NODE_NAME "FineMote"
#endif

#include "BSP_POSIX.h"

template <typename = void>
struct posix_ready : std::false_type {};

template <>
struct posix_ready<std::void_t<
    decltype(::pthread_create(std::declval<pthread_t*>(), std::declval<const pthread_attr_t*>(), std::declval<void* (*)(void*)>(), std::declval<void*>())),
    decltype(::clock_gettime(0, std::declval<struct timespec*>())),
    decltype(::usleep(0u)),
    decltype(::sleep(0u))
>> : std::true_type {};

inline constexpr bool microros_supported = posix_ready<>::value;

static_assert(microros_supported,
    "MicroROS is disabled: this BSP lacks a POSIX compatibility layer. "
    "Link a POSIX library (e.g. FreeRTOS-POSIX on STM32) and ensure its headers are in the include path."
);

template <typename = void>
class MicroROS_Manager;

template <typename = void>
class ROSAgent;

template <>
class ROSAgent<std::enable_if_t<microros_supported>> {
public:
    ROSAgent();

    virtual ~ROSAgent() = default;

    virtual bool Init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) = 0;
    virtual void Execute() = 0;
    virtual void Fini() = 0;
};

template<typename T, typename = void>
struct has_GetRosBinder: std::false_type {};

template<typename T>
struct has_GetRosBinder<T, std::void_t<decltype(std::declval<T&>().GetRosBinder())>>: std::true_type {};

template<typename T>
inline constexpr bool has_GetRosBinder_v = has_GetRosBinder<T>::value;

template<typename>
inline constexpr bool dependent_false_v = false;

template<typename T, typename = void>
struct callback_message_type {
    static_assert(
        dependent_false_v<T>,
        "Unsupported callback type. Expected a non-generic callable with exactly one message argument."
    );
};

template<typename C, typename Ret, typename Arg>
struct callback_message_type<Ret (C::*)(Arg) const, void> {
    using type = std::remove_cv_t<std::remove_reference_t<Arg>>;
};

template<typename T>
struct callback_message_type<T, std::void_t<decltype(&std::decay_t<T>::operator())>>:
    callback_message_type<decltype(&std::decay_t<T>::operator())> {};

template<typename T>
using callback_message_type_t = typename callback_message_type<std::decay_t<T>>::type;

template<typename MsgT>
class RosPublisher: public ROSAgent<> {
    static_assert(
        RosMsgTraits<MsgT>::registered,
        "RosPublisher<MsgT>: MsgT is not registered. Please use DEFINE_MICROROS_MSG(...) first."
    );

public:
    using ConverterFunc = std::function<void(MsgT&)>;

    template<typename ObjT, std::enable_if_t<has_GetRosBinder_v<ObjT>, int> = 0>
    RosPublisher(const char* obj_name, ObjT& obj):
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + obj_name + "/" + RosMsgTraits<MsgT>::name),
        converter_(obj.GetRosBinder()) {}

    template<typename FuncT, std::enable_if_t<!has_GetRosBinder_v<std::decay_t<FuncT>>, int> = 0>
    RosPublisher(const char* base_name, FuncT&& func):
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name + "/" + RosMsgTraits<MsgT>::name),
        converter_(std::forward<FuncT>(func)) {}

    bool Init(rcl_node_t* node, rclc_support_t*, rclc_executor_t*) final {
        const auto* type_support = RosMsgTraits<MsgT>::GetTypeSupport();
        const rcl_ret_t ret = rclc_publisher_init_default(&publisher_, node, type_support, topic_str_.c_str());
        if (ret == RCL_RET_OK) {
            node_ = node;
            return true;
        }
        return false;
    }

    void Execute() final {
        converter_(msg_);
        [[maybe_unused]] const rcl_ret_t result = rcl_publish(&publisher_, &msg_, nullptr);
    }

    void Fini() final {
        if (node_ != nullptr) {
            [[maybe_unused]] const rcl_ret_t result = rcl_publisher_fini(&publisher_, node_);
            publisher_ = rcl_get_zero_initialized_publisher();
            node_ = nullptr;
        }
    }

private:
    std::string topic_str_;
    ConverterFunc converter_;
    rcl_publisher_t publisher_ { rcl_get_zero_initialized_publisher() };
    rcl_node_t* node_ = nullptr;
    MsgT msg_ {};
};

template<typename ObjT, std::enable_if_t<has_GetRosBinder_v<ObjT>, int> = 0>
RosPublisher(const char*, ObjT&)
    -> RosPublisher<callback_message_type_t<decltype(std::declval<ObjT&>().GetRosBinder())>>;

template<typename FuncT, std::enable_if_t<!has_GetRosBinder_v<std::decay_t<FuncT>>, int> = 0>
RosPublisher(const char*, FuncT&&) -> RosPublisher<callback_message_type_t<FuncT>>;

template<typename MsgT>
class RosSubscriber: public ROSAgent<> {
    static_assert(
        RosMsgTraits<MsgT>::registered,
        "RosSubscriber<MsgT>: MsgT is not registered. Please use DEFINE_MICROROS_MSG(...) first."
    );

public:
    using CallbackFunc = std::function<void(const MsgT&)>;

    template<typename FuncT>
    RosSubscriber(const char* base_name, FuncT&& callback):
        callback_(std::forward<FuncT>(callback)),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name) {}

    bool Init(rcl_node_t* node, rclc_support_t*, rclc_executor_t* executor) final {
        const auto* type_support = RosMsgTraits<MsgT>::GetTypeSupport();
        rcl_ret_t ret = rclc_subscription_init_best_effort(&subscriber_, node, type_support, topic_str_.c_str());
        if (ret != RCL_RET_OK) {
            return false;
        }
        node_ = node;

        auto callback_wrapper = [](const void* msgin, void* untyped_self) {
            auto* self = static_cast<RosSubscriber*>(untyped_self);
            const auto* concrete_msg = static_cast<const MsgT*>(msgin);
            self->callback_(*concrete_msg);
        };

        ret = rclc_executor_add_subscription_with_context(
            executor,
            &subscriber_,
            &msg_,
            callback_wrapper,
            this,
            ON_NEW_DATA
        );
        if (ret != RCL_RET_OK) {
            Fini();
            return false;
        }
        return true;
    }

    void Execute() final {}

    void Fini() final {
        if (node_ != nullptr) {
            [[maybe_unused]] const rcl_ret_t result = rcl_subscription_fini(&subscriber_, node_);
            subscriber_ = rcl_get_zero_initialized_subscription();
            node_ = nullptr;
        }
    }

private:
    CallbackFunc callback_;
    std::string topic_str_;
    rcl_subscription_t subscriber_ { rcl_get_zero_initialized_subscription() };
    rcl_node_t* node_ = nullptr;
    MsgT msg_ {};
};

template<typename FuncT>
RosSubscriber(const char*, FuncT&&) -> RosSubscriber<callback_message_type_t<FuncT>>;

#endif
