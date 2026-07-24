/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_MANAGER_HPP
#define FINEMOTE_MICROROS_MANAGER_HPP

#include "Board.h"
#include "etl/list.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "MicroROS_Agent.hpp"
#include "MicroROS_Transport.hpp"

constexpr size_t MICROROS_MAX_HANDLES = 10;
constexpr size_t MICROROS_MAX_AGENTS = 10;

template <typename>
class MicroROS_Manager;

template <>
class MicroROS_Manager<std::enable_if_t<microros_supported>>
{
public:
    enum class State { WAITING_AGENT, INITIALIZING, RUNNING, ERROR };

    static MicroROS_Manager& GetInstance()
    {
        static MicroROS_Manager instance;
        return instance;
    }

    void RegisterAgent(ROSAgent<>* agent)
    {
        if (!agents_.full())
        {
            agents_.push_back(agent);
        }
    }

    void Handle()
    {
        switch (state_)
        {
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
    MicroROS_Manager()
    {
        MicroROS_Transport::GetInstance();
        rmw_uros_set_custom_transport(true, nullptr,
                                      MicroROS_Transport::Open,
                                      MicroROS_Transport::Close,
                                      MicroROS_Transport::Write,
                                      MicroROS_Transport::Read);
        allocator_ = rcl_get_default_allocator();
        StartThread();
    }

    void StartThread()
    {
        pthread_attr_t attr;
        pthread_attr_init(&attr);
        pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED);

        constexpr size_t STACK_SIZE = 20 * 1024;
        pthread_attr_setstacksize(&attr, STACK_SIZE);

        if (pthread_create(&thread_, &attr, &MicroROS_Manager::ThreadFunc, this) != 0)
        {
            HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, GPIO_PIN_RESET);
        }
        pthread_attr_destroy(&attr);
    }

    [[noreturn]] static void* ThreadFunc(void* arg)
    {
        auto* manager = static_cast<MicroROS_Manager*>(arg);

        for (;;)
        {
            manager->Handle();
            usleep(200000);
        }
    }

    ~MicroROS_Manager() = default;

    void HandleWaiting()
    {
        if (rmw_uros_ping_agent(500, 1) == RMW_RET_OK)
        {
            state_ = State::INITIALIZING;
        }
    }

    void HandleInitializing()
    {
        rcl_ret_t ret;

        ret = rclc_support_init(&support_, 0, nullptr, &allocator_);
        if (ret != RCL_RET_OK)
        {
            GotoError();
            return;
        }

        ret = rclc_node_init_default(&node_, MICROROS_NODE_NAME, "", &support_);
        if (ret != RCL_RET_OK)
        {
            GotoError();
            return;
        }

        ret = rclc_executor_init(&executor_, &support_.context, MICROROS_MAX_HANDLES, &allocator_);
        if (ret != RCL_RET_OK)
        {
            GotoError();
            return;
        }

        for (auto* agent : agents_)
        {
            if (!agent->Init(&node_, &support_, &executor_))
            {
                GotoError();
                return;
            }
        }

        ping_counter_ = 0;
        state_ = State::RUNNING;
    }

    void HandleRunning()
    {
        rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(10));

        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT)
        {
            GotoError();
            return;
        }

        for (auto* agent : agents_)
        {
            agent->Execute();
        }

        if (++ping_counter_ >= 5)  // 5 * 200ms = 1000ms
        {
            ping_counter_ = 0;
            if (rmw_uros_ping_agent(500, 1) != RMW_RET_OK)
            {
                GotoError();
            }
        }
    }

    void GotoError()
    {
        Cleanup();
        state_ = State::ERROR;
    }

    void Cleanup()
    {
        for (auto* agent : agents_)
        {
            agent->Fini();
        }
        (void)rclc_executor_fini(&executor_);
        (void)rcl_node_fini(&node_);
        (void)rclc_support_fini(&support_);
    }

    State state_ = State::WAITING_AGENT;
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;
    uint32_t ping_counter_ = 0;
    etl::list<ROSAgent<>*, MICROROS_MAX_AGENTS> agents_;
    pthread_t thread_{};
};

inline ROSAgent<std::enable_if_t<microros_supported>>::ROSAgent()
{
    MicroROS_Manager<>::GetInstance().RegisterAgent(this);
}

#endif
