/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_MICROROS_BASE_HPP
#define FINEMOTE_MICROROS_BASE_HPP

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <rosidl_runtime_c/message_type_support_struct.h>
#include <functional>
#include <concepts>

template<typename T> struct RosMsgTypeTraits {
    static const rosidl_message_type_support_t* GetTypeSupport() { return nullptr; }
};

#define DEFINE_MICROROS_MSG_TYPE(CppType, PkgName, MsgSub, MsgName) \
    template<> struct RosMsgTypeTraits<CppType> { \
        static const rosidl_message_type_support_t* GetTypeSupport() { \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName); \
        } \
    };

class ROSAgent {
public:
    ROSAgent() {

        this->next_ = head_;
        head_ = this;
    }

    virtual ~ROSAgent() = default;

    virtual bool Init(rcl_node_t* node, rclc_support_t* support) = 0;

    virtual bool AddToExecutor(rclc_executor_t* executor) { return true; }

    virtual size_t GetHandleCount() const { return 0; }

    virtual void Reset() = 0;

    virtual rcl_timer_t* GetTimerHandle() { return nullptr; }

    static ROSAgent* GetHead() { return head_; }
    ROSAgent* GetNext() const { return next_; }

protected:
    static ROSAgent* head_;

    ROSAgent* next_;
};

template<typename MsgT>
class RosPublisher : public ROSAgent {
public:
    RosPublisher(const char* topic, bool best_effort = true)
        : topic_(topic), best_effort_(best_effort), initialized_(false) {}

    MsgT& load_msg() { return msg_; }

    void publish() {
        if (initialized_) {
            rcl_publish(&pub_, &msg_, nullptr);
        }
    }

    void publish(const MsgT& external_msg) {
        if (initialized_) {
            rcl_publish(&pub_, &external_msg, nullptr);
        }
    }

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        const auto* ts = RosMsgTypeTraits<MsgT>::GetTypeSupport();
        if (!ts) return false;

        rcl_ret_t rc = best_effort_
            ? rclc_publisher_init_best_effort(&pub_, node, ts, topic_)
            : rclc_publisher_init_default(&pub_, node, ts, topic_);

        initialized_ = (rc == RCL_RET_OK);
        return initialized_;
    }

    void Reset() override {
        if (initialized_) {
            rcl_publisher_fini(&pub_, nullptr);
            initialized_ = false;
        }
    }

private:
    const char* topic_;
    bool best_effort_;
    bool initialized_;
    rcl_publisher_t pub_;
    MsgT msg_;
};

template<typename MsgT>
class RosSubscriber : public ROSAgent {
public:
    using CallbackFunc = std::function<void(const MsgT&)>;

    RosSubscriber(const char* topic, CallbackFunc callback, bool best_effort = true)
        : topic_(topic), callback_(callback), best_effort_(best_effort), initialized_(false) {}

    size_t GetHandleCount() const override { return 1; }

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        const auto* ts = RosMsgTypeTraits<MsgT>::GetTypeSupport();
        if (!ts) return false;

        rcl_ret_t rc = best_effort_
            ? rclc_subscription_init_best_effort(&sub_, node, ts, topic_)
            : rclc_subscription_init_default(&sub_, node, ts, topic_);

        initialized_ = (rc == RCL_RET_OK);
        return initialized_;
    }

    bool AddToExecutor(rclc_executor_t* executor) override {
        if (!initialized_) return false;

        return (rclc_executor_add_subscription_with_context(
            executor, &sub_, &msg_, &StaticCallback, this, ON_NEW_DATA
        ) == RCL_RET_OK);
    }

    void Reset() override {
        if (initialized_) {
            rcl_subscription_fini(&sub_, nullptr);
            initialized_ = false;
        }
    }

private:
    static void StaticCallback(const void * msgin, void * context) {
        auto* self = static_cast<RosSubscriber<MsgT>*>(context);
        if (self && self->callback_) {
            self->callback_(self->msg_);
        }
    }

    const char* topic_;
    CallbackFunc callback_;
    bool best_effort_;
    bool initialized_;
    rcl_subscription_t sub_;
    MsgT msg_;
};

class Timer : public ROSAgent {
public:
    using TimerCallback = std::function<void()>;

    Timer(unsigned int period_ms, TimerCallback callback)
        : period_ms_(period_ms), callback_(callback), initialized_(false) {
    }

    size_t GetHandleCount() const override { return 1; }

    rcl_timer_t* GetTimerHandle() override { return &timer_; }

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        rcl_ret_t rc = rclc_timer_init_default(
            &timer_,
            support,
            RCL_MS_TO_NS(period_ms_),
            StaticTimerCallback
        );
        initialized_ = (rc == RCL_RET_OK);
        return initialized_;
    }

    bool AddToExecutor(rclc_executor_t* executor) override {
        if (!initialized_) return false;
        return (rclc_executor_add_timer(executor, &timer_) == RCL_RET_OK);
    }

    void Reset() override {
        if (initialized_) {
            rcl_timer_fini(&timer_);
            initialized_ = false;
        }
    }

private:
    static void StaticTimerCallback(rcl_timer_t * timer, int64_t last_call_time) {
        (void)last_call_time;

        ROSAgent* curr = ROSAgent::GetHead();
        while (curr) {
            if (curr->GetTimerHandle() == timer) {
                static_cast<Timer*>(curr)->callback_();
                return;
            }
            curr = curr->GetNext();
        }
    }

    unsigned int period_ms_;
    TimerCallback callback_;
    rcl_timer_t timer_;
    bool initialized_;
};

struct DisableRos {
    static constexpr bool enabled = false;
    template<typename StateT> void Update(const StateT&) const {}
};

template<typename _MsgT, typename _StateT>
class EnableRosPublisher {
public:
    static constexpr bool enabled = true;
    using MsgT = _MsgT;
    using StateT = _StateT;
    using ConverterFunc = std::function<void(MsgT&, const StateT&)>;

    EnableRosPublisher(const char* topic, ConverterFunc converter)
        : publisher_(topic), converter_(converter) {}

    void Update(const StateT& state) {
        MsgT& msg = publisher_.load_msg();
        if (converter_) converter_(msg, state);
        publisher_.publish();
    }

private:
    RosPublisher<MsgT> publisher_;
    ConverterFunc converter_;
};

#endif // FINEMOTE_MICROROS_BASE_HPP