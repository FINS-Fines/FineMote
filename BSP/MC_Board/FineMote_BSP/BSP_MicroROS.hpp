/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_BSP_MICROROS_HPP
#define FINEMOTE_BSP_MICROROS_HPP

 #include <functional>

#include "cmsis_os.h"
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <rosidl_runtime_c/message_type_support_struct.h>

template <typename T>
struct RosMsgTypeTraits {
  static const rosidl_message_type_support_t* GetTypeSupport() {
    return nullptr;
  }
};

#define DEFINE_MICROROS_MSG_TYPE(CppType, PkgName, MsgSub, MsgName)          \
  template <>                                                                \
  struct RosMsgTypeTraits<CppType> {                                         \
    static const rosidl_message_type_support_t* GetTypeSupport() {           \
      return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName);          \
    }                                                                        \
  };

template <bool enable = true> class MicroROS_Base;

template <bool enable = true>
class ROSAgent {
public:
    ROSAgent() { MicroROS_Base<enable>::GetInstance().RegisterAgent(this); }
    virtual ~ROSAgent() = default;
    virtual bool Init(rcl_node_t* node, rclc_support_t* support) = 0;
    virtual bool AddToExecutor(rclc_executor_t* executor) { return true; }
    virtual size_t GetHandleCount() const { return 0; }
    virtual void Execute() = 0;
    virtual void Reset() = 0;
    virtual rcl_timer_t* GetTimerHandle() { return nullptr; }
protected:
    bool initialized_ = false;
};

template <typename T, typename = void>
struct has_GetRosBinder : std::false_type {};

template <typename T>
struct has_GetRosBinder<T, std::void_t<decltype(std::declval<T&>().GetRosBinder())>>
    : std::true_type {};

template <typename T>
struct msg_traits : msg_traits<decltype(&T::operator())> {};

template <typename C, typename Ret, typename MsgT>
struct msg_traits<Ret(C::*)(MsgT&) const> {
    using type = std::remove_cv_t<std::remove_reference_t<MsgT>>;
};

template <typename MsgT>
class RosPublisher : public ROSAgent<> {
 public:
  using ConverterFunc = std::function<void(MsgT&)>;

    template <typename ObjT,
                  typename = std::enable_if_t<has_GetRosBinder<ObjT>::value>>
        RosPublisher(const char* topic_name, ObjT& obj)
            : topic_name_(topic_name) {
        this->converter_ = obj.GetRosBinder();
    }

    template <typename FuncT,
              typename = std::enable_if_t<!has_GetRosBinder<FuncT>::value>>
    RosPublisher(const char* topic_name, FuncT&& func)
        : topic_name_(topic_name), converter_(std::forward<FuncT>(func)) {}

    bool Init(rcl_node_t* node, rclc_support_t* support) override {
        const auto* type_support = RosMsgTypeTraits<MsgT>::GetTypeSupport();
        if (!type_support) return false;

        rcl_ret_t ret = rclc_publisher_init_best_effort(
            &publisher_, node, type_support, topic_name_);
        this->initialized_ = (ret == RCL_RET_OK);
        return this->initialized_;
    }

    void Execute() override {
        if (!this->initialized_ || !converter_) return;
        converter_(msg_);
        rcl_publish(&publisher_, &msg_, nullptr);
    }

    void Reset() override {
        if (this->initialized_) {
            rcl_publisher_fini(&publisher_, nullptr);
            this->initialized_ = false;
        }
    }
 private:
  const char* topic_name_;
  ConverterFunc converter_;
  rcl_publisher_t publisher_;
  MsgT msg_{};
};

template <typename ObjT, typename = std::enable_if_t<has_GetRosBinder<ObjT>::value>>
RosPublisher(const char*, ObjT&) -> RosPublisher<
    typename msg_traits<decltype(std::declval<ObjT>().GetRosBinder())>::type>;

template <typename FuncT, typename = std::enable_if_t<!has_GetRosBinder<FuncT>::value>>
RosPublisher(const char*, FuncT) -> RosPublisher<
    typename msg_traits<FuncT>::type>;

template <typename MsgT>
class RosSubscriber : public ROSAgent<> {
 public:
  using CallbackFunc = std::function<void(const MsgT&)>;

  RosSubscriber(const char* topic_name, CallbackFunc callback)
      : topic_name_(topic_name), callback_(callback) {}

  size_t GetHandleCount() const override { return 1; }

  bool Init(rcl_node_t* node, rclc_support_t* support) override {
    const auto* type_support = RosMsgTypeTraits<MsgT>::GetTypeSupport();
    if (!type_support) {
      return false;
    }

    rcl_ret_t ret = rclc_subscription_init_best_effort(
        &subscriber_, node, type_support, topic_name_);
    this->initialized_ = (ret == RCL_RET_OK);
    return this->initialized_;
  }

  bool AddToExecutor(rclc_executor_t* executor) override {
    if (!this->initialized_) {
      return false;
    }

    rcl_ret_t ret = rclc_executor_add_subscription_with_context(
        executor, &subscriber_, &msg_, &StaticCallback, this, ON_NEW_DATA);
    return (ret == RCL_RET_OK);
  }

  void Execute() override {}

  void Reset() override {
    if (this->initialized_) {
      rcl_subscription_fini(&subscriber_, nullptr);
      this->initialized_ = false;
    }
  }

 private:
    static void StaticCallback(const void* msgin, void* untyped_self) {
      auto* self = static_cast<RosSubscriber*>(untyped_self);
      auto* concrete_msg = static_cast<const MsgT*>(msgin);
      if (self && self->callback_) self->callback_(*concrete_msg);
  }

  const char* topic_name_;
  CallbackFunc callback_;
  rcl_subscription_t subscriber_;
  MsgT msg_{};
};

template <typename FuncT>
RosSubscriber(const char*, FuncT) -> RosSubscriber<typename msg_traits<FuncT>::type>;

template<bool enable>
class Timer : public ROSAgent<> {
 public:
  using TimerCallback = std::function<void()>;

  Timer(unsigned int period_ms, TimerCallback callback)
      : period_ms_(period_ms), callback_(callback) {}

  size_t GetHandleCount() const override { return 1; }

  rcl_timer_t* GetTimerHandle() override { return &timer_; }

  bool Init(rcl_node_t* node, rclc_support_t* support) override {
    rcl_ret_t ret = rclc_timer_init_default(
        &timer_, support, RCL_MS_TO_NS(period_ms_), StaticTimerCallback);
    this->initialized_ = (ret == RCL_RET_OK);
    return this->initialized_;
  }

  bool AddToExecutor(rclc_executor_t* executor) override {
    if (!this->initialized_) {
      return false;
    }
    return (rclc_executor_add_timer(executor, &timer_) == RCL_RET_OK);
  }

  void Execute() override {}

  void Reset() override {
    if (this->initialized_) {
      rcl_timer_fini(&timer_);
      this->initialized_ = false;
    }
  }

 private:
    static void StaticTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
        (void)last_call_time;
        auto& agents = MicroROS_Base<enable>::GetInstance().GetAgents();
        for (auto* agent : agents) {
            if (agent->GetTimerHandle() == timer) {
                static_cast<Timer*>(agent)->callback_();
                return;
            }
        }
    }

  unsigned int period_ms_;
  TimerCallback callback_;
  rcl_timer_t timer_;
};

#endif

