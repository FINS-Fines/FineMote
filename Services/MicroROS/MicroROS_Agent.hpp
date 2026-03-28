/******************************************************************************
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

#include "cmsis_os.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

#ifndef MICROROS_NODE_NAME
#define MICROROS_NODE_NAME "FineMote"
#endif

template <typename>
inline constexpr bool dependent_false_v = false;

template <typename T>
struct RosMsgTraits
{
    static constexpr bool registered = false;
};

#define DEFINE_MICROROS_MSG(CppType, PkgName, MsgSub, MsgName) \
    template<> \
    struct RosMsgTraits<CppType> { \
        static constexpr bool registered = true; \
        static constexpr const char* name = #MsgName; \
        static const rosidl_message_type_support_t* GetTypeSupport() { \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName); \
        } \
    };

#define PUBLISHER(obj) RosPublisher(#obj, obj)

template <bool enable = true>
class MicroROS_Manager;

template <bool enable = true>
class ROSAgent
{
public:
    ROSAgent()
    {
        MicroROS_Manager<enable>::GetInstance().RegisterAgent(this);
    }

    virtual ~ROSAgent() = default;

    virtual bool Init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) = 0;
    virtual void Execute() = 0;
    virtual void Final() = 0;
};

template <typename T, typename = void>
struct has_GetRosBinder : std::false_type
{
};

template <typename T>
struct has_GetRosBinder<T, std::void_t<decltype(std::declval<T&>().GetRosBinder())>> : std::true_type
{
};

template <typename T>
inline constexpr bool has_GetRosBinder_v = has_GetRosBinder<T>::value;

template <typename T, typename = void>
struct callback_message_type
{
    static_assert(
        dependent_false_v<T>,
        "Unsupported callback type. "
        "Expected a non-generic callable with exactly one message argument."
    );
};

template <typename C, typename Ret, typename Arg>
struct callback_message_type<Ret (C::*)(Arg) const, void>
{
    using type = std::remove_cv_t<std::remove_reference_t<Arg>>;
};

template <typename T>
struct callback_message_type<T, std::void_t<decltype(&std::decay_t<T>::operator())>> :
    callback_message_type<decltype(&std::decay_t<T>::operator())>
{
};

template <typename T>
using callback_message_type_t = typename callback_message_type<std::decay_t<T>>::type;

template <typename MsgT>
class RosPublisher : public ROSAgent<>
{
    static_assert(
        RosMsgTraits<MsgT>::registered,
        "RosPublisher<MsgT>: MsgT is not registered. "
        "Please use DEFINE_MICROROS_MSG(...) first."
    );

public:
    using ConverterFunc = std::function<void(MsgT&)>;

    template <typename ObjT, std::enable_if_t<has_GetRosBinder_v<ObjT>, int> = 0>
    RosPublisher(const char* obj_name, ObjT& obj) :
        converter_(obj.GetRosBinder()),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + obj_name + "/" + RosMsgTraits<MsgT>::name)
    {
    }

    template <typename FuncT, std::enable_if_t<!has_GetRosBinder_v<std::decay_t<FuncT>>, int> = 0>
    RosPublisher(const char* base_name, FuncT&& func) :
        converter_(std::forward<FuncT>(func)),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name + "/" + RosMsgTraits<MsgT>::name)
    {
    }

    bool Init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) final
    {
        const auto* type_support = RosMsgTraits<MsgT>::GetTypeSupport();
        rcl_ret_t ret = rclc_publisher_init_best_effort(&publisher_, node, type_support, topic_str_.c_str());
        return (ret == RCL_RET_OK);
    }

    void Execute() final
    {
        converter_(msg_);
        (void)rcl_publish(&publisher_, &msg_, nullptr);
    }

    void Final() final
    {
        (void)rcl_publisher_fini(&publisher_, nullptr);
    }

private:
    std::string topic_str_;
    ConverterFunc converter_;
    rcl_publisher_t publisher_{rcl_get_zero_initialized_publisher()};
    MsgT msg_{};
};

template <typename ObjT, std::enable_if_t<has_GetRosBinder_v<ObjT>, int> = 0>
RosPublisher(const char*, ObjT&)
    -> RosPublisher<callback_message_type_t<decltype(std::declval<ObjT&>().GetRosBinder())>>;

template <typename FuncT, std::enable_if_t<!has_GetRosBinder_v<std::decay_t<FuncT>>, int> = 0>
RosPublisher(const char*, FuncT&&) -> RosPublisher<callback_message_type_t<FuncT>>;

template <typename MsgT>
class RosSubscriber : public ROSAgent<>
{
    static_assert(
        RosMsgTraits<MsgT>::registered,
        "RosSubscriber<MsgT>: MsgT is not registered. "
        "Please use DEFINE_MICROROS_MSG(...) first."
    );

public:
    using CallbackFunc = std::function<void(const MsgT&)>;

    template <typename FuncT>
    RosSubscriber(const char* base_name, FuncT&& callback) :
        callback_(std::forward<FuncT>(callback)),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name)
    {
    }

    bool Init(rcl_node_t* node, rclc_support_t* support, rclc_executor_t* executor) final
    {
        const auto* type_support = RosMsgTraits<MsgT>::GetTypeSupport();

        rcl_ret_t ret = rclc_subscription_init_best_effort(&subscriber_, node, type_support, topic_str_.c_str());
        if (ret != RCL_RET_OK)
        {
            return false;
        }

        ret = rclc_executor_add_subscription_with_context(
            executor,
            &subscriber_,
            &msg_,
            &StaticCallback,
            this,
            ON_NEW_DATA
        );
        return (ret == RCL_RET_OK);
    }

    void Execute() final
    {
    }

    void Final() final
    {
        (void)rcl_subscription_fini(&subscriber_, nullptr);
    }

private:
    static void StaticCallback(const void* msgin, void* untyped_self)
    {
        auto* self = static_cast<RosSubscriber*>(untyped_self);
        auto* concrete_msg = static_cast<const MsgT*>(msgin);
        self->callback_(*concrete_msg);
    }

    std::string topic_str_;
    CallbackFunc callback_;
    rcl_subscription_t subscriber_{rcl_get_zero_initialized_subscription()};
    MsgT msg_{};
};

template <typename FuncT>
RosSubscriber(const char*, FuncT&&) -> RosSubscriber<callback_message_type_t<FuncT>>;

#endif
