/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_MANAGER_HPP
#define FINEMOTE_MICROROS_MANAGER_HPP

#include "Board.h"
#include "etl/list.h"

#include <cstring>

#include <FreeRTOS_POSIX.h>
#include <FreeRTOS_POSIX/pthread.h>
#include <FreeRTOS_POSIX/unistd.h>
#include <atomic>

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "Bus/UART_Base.hpp"
#include "MicroROS_Agent.hpp"
#include "MicroROS_Transport.hpp"

constexpr size_t MICROROS_MAX_HANDLES = 10;
constexpr size_t MICROROS_MAX_AGENTS = 10;

template <bool enable>
class MicroROS_Manager
{
    friend class MicroROS_Transport<enable>;

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
    MicroROS_Manager() :
        dma_buffer_([this](uint8_t* data, size_t size)
        {
            this->PushRxData(data, size);
        })
    {
        Setup();
        StartThread();
    }

    void Setup()
    {
        allocator_ = rcl_get_default_allocator();

        UART_Base<MICRO_ROS_UART_ID>::GetInstance().BindTxHandle([this]()
        {
            this->tx_busy_.store(false, std::memory_order_release);
            return true;
        });

        rmw_uros_set_custom_transport(true, nullptr, MicroROS_Transport<enable>::Open,
                                      MicroROS_Transport<enable>::Close, MicroROS_Transport<enable>::Write,
                                      MicroROS_Transport<enable>::Read);
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

    void PushRxData(uint8_t* data, size_t size)
    {
        if (size == 0) return;

        size_t w = rx_w_.load(std::memory_order_relaxed);
        size_t r = rx_r_.load(std::memory_order_acquire);
        size_t space = MICROROS_BUF_SIZE - (w - r);

        if (size > space) return;

        size_t pos = w & (MICROROS_BUF_SIZE - 1);
        size_t first = MICROROS_BUF_SIZE - pos;
        if (size <= first) {
            memcpy(&rx_buf_[pos], data, size);
        } else {
            memcpy(&rx_buf_[pos], data, first);
            memcpy(&rx_buf_[0], data + first, size - first);
        }
        rx_w_.store(w + size, std::memory_order_release);
    }

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
    uint8_t rx_buf_[MICROROS_BUF_SIZE];
    std::atomic<size_t> rx_w_{0};
    std::atomic<size_t> rx_r_{0};

    etl::list<ROSAgent<>*, MICROROS_MAX_AGENTS> agents_;

    uint8_t tx_buffer_[MICROROS_BUF_SIZE];
    std::atomic<bool> tx_busy_{false};
    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;

    pthread_t thread_{};
};

template <>
class MicroROS_Manager<false>
{
public:
    static MicroROS_Manager& GetInstance()
    {
        static_assert(
            WITH_MICRO_ROS,
            "MicroROS is disabled in Board.h. Please set WITH_MICRO_ROS = true to use MicroROS_Base."
        );
        static MicroROS_Manager instance;
        return instance;
    }
};

#endif
