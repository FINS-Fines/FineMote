/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_MANAGER_HPP
#define FINEMOTE_MICROROS_MANAGER_HPP

#include <cstddef>
#include <cstdint>

#include "BSP_POSIX.h"

#include <etl/list.h>
#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>

#include "MicroROS/MicroROS_Agent.hpp"
#include "MicroROS/MicroROS_Backend.hpp"

constexpr size_t MICROROS_MAX_HANDLES = 10;
constexpr size_t MICROROS_MAX_AGENTS = 10;
template <>
class MicroROS_Manager<std::enable_if_t<microros_supported>> {
public:
    enum class State { WAITING_AGENT, INITIALIZING, RUNNING, ERROR };

    static MicroROS_Manager& GetInstance() {
        static MicroROS_Manager instance;
        return instance;
    }

    void RegisterAgent(ROSAgent<>* agent) {
        const bool registration_open = !started_ || state_ == State::WAITING_AGENT;
        if (registration_open && !agents_.full()) {
            agents_.push_back(agent);
        }
    }

    bool Start(MicroROS_Backend& backend) {
        if (started_) {
            return true;
        }

        if (!backend.Prepare()) {
            return false;
        }

        init_options_ = rcl_get_zero_initialized_init_options();
        if (rcl_init_options_init(&init_options_, allocator_) != RCL_RET_OK) {
            return false;
        }
        init_options_initialized_ = true;

        rmw_options_ = rcl_init_options_get_rmw_init_options(&init_options_);
        if (rmw_options_ == nullptr || backend.Configure(rmw_options_) != RMW_RET_OK) {
            ResetInitOptions();
            return false;
        }

        state_ = State::WAITING_AGENT;
        started_ = true;
        if (!StartThread()) {
            started_ = false;
            ResetInitOptions();
            return false;
        }

        return true;
    }

    void Handle() {
        if (!started_) {
            return;
        }

        switch (state_) {
            case State::WAITING_AGENT:
                HandleWaiting();
                break;
            case State::INITIALIZING:
                HandleInitializing();
                break;
            case State::RUNNING:
                HandleRunning();
                break;
            case State::ERROR:
                sleep(1);
                state_ = State::WAITING_AGENT;
                break;
        }
    }

private:
    MicroROS_Manager(): allocator_(rcl_get_default_allocator()) {}

    bool StartThread() {
        pthread_attr_t attr;
        if (pthread_attr_init(&attr) != 0) {
            return false;
        }

        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);
        constexpr size_t STACK_SIZE = 20 * 1024;
        pthread_attr_setstacksize(&attr, STACK_SIZE);

        const int result = pthread_create(&thread_, &attr, &MicroROS_Manager::ThreadFunc, this);
        pthread_attr_destroy(&attr);
        return result == 0;
    }

    [[noreturn]] static void* ThreadFunc(void* arg) {
        auto* manager = static_cast<MicroROS_Manager*>(arg);

        for (;;) {
            manager->Handle();
            usleep(200000);
        }
    }

    ~MicroROS_Manager() = default;

    void HandleWaiting() {
        if (rmw_uros_ping_agent_options(500, 1, rmw_options_) == RMW_RET_OK) {
            state_ = State::INITIALIZING;
        }
    }

    void HandleInitializing() {
        ResetRosEntities();

        if (rclc_support_init_with_options(&support_, 0, nullptr, &init_options_, &allocator_) != RCL_RET_OK) {
            GotoError();
            return;
        }
        support_initialized_ = true;

        if (rclc_node_init_default(&node_, MICROROS_NODE_NAME, "", &support_) != RCL_RET_OK) {
            GotoError();
            return;
        }
        node_initialized_ = true;

        if (rclc_executor_init(&executor_, &support_.context, MICROROS_MAX_HANDLES, &allocator_) != RCL_RET_OK) {
            GotoError();
            return;
        }
        executor_initialized_ = true;

        for (auto* agent: agents_) {
            if (!agent->Init(&node_, &support_, &executor_)) {
                GotoError();
                return;
            }
            ++initialized_agents_;
        }

        ping_counter_ = 0;
        state_ = State::RUNNING;
    }

    void HandleRunning() {
        const rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));
        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            GotoError();
            return;
        }

        for (auto* agent: agents_) {
            agent->Execute();
        }

        if (++ping_counter_ >= 50) {
            ping_counter_ = 0;
            if (rmw_uros_ping_agent_options(1000, 1, rmw_options_) != RMW_RET_OK) {
                GotoError();
            }
        }
    }

    void GotoError() {
        Cleanup();
        state_ = State::ERROR;
    }

    void Cleanup() {
        size_t agent_index = 0;
        for (auto* agent: agents_) {
            if (agent_index++ >= initialized_agents_) {
                break;
            }
            agent->Fini();
        }
        initialized_agents_ = 0;

        if (executor_initialized_) {
            [[maybe_unused]] const rcl_ret_t result = rclc_executor_fini(&executor_);
            executor_initialized_ = false;
        }
        if (node_initialized_) {
            [[maybe_unused]] const rcl_ret_t result = rcl_node_fini(&node_);
            node_initialized_ = false;
        }
        if (support_initialized_) {
            [[maybe_unused]] const rcl_ret_t result = rclc_support_fini(&support_);
            support_initialized_ = false;
        }
    }

    void ResetRosEntities() {
        support_ = {};
        node_ = rcl_get_zero_initialized_node();
        executor_ = rclc_executor_get_zero_initialized_executor();
        initialized_agents_ = 0;
    }

    void ResetInitOptions() {
        if (init_options_initialized_) {
            [[maybe_unused]] const rcl_ret_t result = rcl_init_options_fini(&init_options_);
            init_options_initialized_ = false;
        }
        init_options_ = rcl_get_zero_initialized_init_options();
        rmw_options_ = nullptr;
    }

    State state_ = State::WAITING_AGENT;
    bool started_ = false;
    bool init_options_initialized_ = false;
    bool support_initialized_ = false;
    bool node_initialized_ = false;
    bool executor_initialized_ = false;
    size_t initialized_agents_ = 0;
    rcl_allocator_t allocator_;
    rcl_init_options_t init_options_ { rcl_get_zero_initialized_init_options() };
    rmw_init_options_t* rmw_options_ = nullptr;
    rclc_support_t support_ {};
    rcl_node_t node_ { rcl_get_zero_initialized_node() };
    rclc_executor_t executor_ { rclc_executor_get_zero_initialized_executor() };
    uint32_t ping_counter_ = 0;
    etl::list<ROSAgent<>*, MICROROS_MAX_AGENTS> agents_;
    pthread_t thread_ {};
};

inline ROSAgent<std::enable_if_t<microros_supported>>::ROSAgent() {
    MicroROS_Manager<>::GetInstance().RegisterAgent(this);
}

#endif
