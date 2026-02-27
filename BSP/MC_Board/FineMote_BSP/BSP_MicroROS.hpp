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

class ROSAgent {
 public:
  ROSAgent() {
    next_ = head_;
    head_ = this;
  }

  virtual ~ROSAgent() = default;

  virtual bool Init(rcl_node_t* node, rclc_support_t* support) = 0;
  virtual bool AddToExecutor(rclc_executor_t* executor) { return true; }
  virtual size_t GetHandleCount() const { return 0; }
  virtual void Execute() = 0;
  virtual void Reset() = 0;
  virtual rcl_timer_t* GetTimerHandle() { return nullptr; }

  static ROSAgent* GetHead() { return head_; }
  ROSAgent* GetNext() const { return next_; }

 protected:
  static ROSAgent* head_;
  ROSAgent* next_ = nullptr;
  bool initialized_ = false;
};

template <typename MsgT, typename StateT>
class RosPublisher : public ROSAgent {
 public:
  static constexpr bool enabled = true;
  using ConverterFunc = std::function<void(MsgT&, const StateT&)>;

  RosPublisher(const char* topic_name, ConverterFunc converter)
      : topic_name_(topic_name), converter_(converter) {}

  void Update(const StateT& state) {
      taskENTER_CRITICAL();
      last_state_ = state;
      taskEXIT_CRITICAL();
    }

    void Execute() override {
      if (!initialized_) {
          return;
      }

      StateT local_state;

      taskENTER_CRITICAL();
      local_state = last_state_;
      taskEXIT_CRITICAL();

      if (converter_) {
          converter_(msg_, local_state);
      }

      rcl_publish(&publisher_, &msg_, nullptr);
  }

  bool Init(rcl_node_t* node, rclc_support_t* support) override {
    const auto* type_support = RosMsgTypeTraits<MsgT>::GetTypeSupport();
    if (!type_support) {
      return false;
    }

    rcl_ret_t ret = rclc_publisher_init_best_effort(
        &publisher_, node, type_support, topic_name_);

    initialized_ = (ret == RCL_RET_OK);
    return initialized_;
  }

  void Reset() override {
    if (initialized_) {
      rcl_publisher_fini(&publisher_, nullptr);
      initialized_ = false;
    }
  }

 private:
  const char* topic_name_;
  ConverterFunc converter_;
  rcl_publisher_t publisher_;
  MsgT msg_{};
  StateT last_state_{};
};

template <typename MsgT>
class RosSubscriber : public ROSAgent {
 public:
  static constexpr bool enabled = true;
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
    initialized_ = (ret == RCL_RET_OK);
    return initialized_;
  }

  bool AddToExecutor(rclc_executor_t* executor) override {
    if (!initialized_) {
      return false;
    }

    rcl_ret_t ret = rclc_executor_add_subscription_with_context(
        executor, &subscriber_, &msg_, &StaticCallback, this, ON_NEW_DATA);
    return (ret == RCL_RET_OK);
  }

  void Execute() override {}

  void Reset() override {
    if (initialized_) {
      rcl_subscription_fini(&subscriber_, nullptr);
      initialized_ = false;
    }
  }

 private:
  static void StaticCallback(const void* msgin, void* context) {
    auto* self = static_cast<RosSubscriber<MsgT>*>(context);
    if (self && self->callback_) {
      self->callback_(self->msg_);
    }
  }

  const char* topic_name_;
  CallbackFunc callback_;
  rcl_subscription_t subscriber_;
  MsgT msg_{};
};

class Timer : public ROSAgent {
 public:
  static constexpr bool enabled = true;
  using TimerCallback = std::function<void()>;

  Timer(unsigned int period_ms, TimerCallback callback)
      : period_ms_(period_ms), callback_(callback) {}

  size_t GetHandleCount() const override { return 1; }

  rcl_timer_t* GetTimerHandle() override { return &timer_; }

  bool Init(rcl_node_t* node, rclc_support_t* support) override {
    rcl_ret_t ret = rclc_timer_init_default(
        &timer_, support, RCL_MS_TO_NS(period_ms_), StaticTimerCallback);
    initialized_ = (ret == RCL_RET_OK);
    return initialized_;
  }

  bool AddToExecutor(rclc_executor_t* executor) override {
    if (!initialized_) {
      return false;
    }
    return (rclc_executor_add_timer(executor, &timer_) == RCL_RET_OK);
  }

  void Execute() override {}

  void Reset() override {
    if (initialized_) {
      rcl_timer_fini(&timer_);
      initialized_ = false;
    }
  }

 private:
  static void StaticTimerCallback(rcl_timer_t* timer, int64_t last_call_time) {
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
};

struct DisableRos {
  static constexpr bool enabled = false;

  void Execute() {}

  template <typename StateT>
  void Update(const StateT&) const {}
};

#endif

