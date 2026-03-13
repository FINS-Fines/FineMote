/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_MICROROS_BASE_HPP
#define FINEMOTE_MICROROS_BASE_HPP

#include "Board.h"
#include "FreeRTOS.h"
#include "cmsis_os.h"
#include "task.h"
#include "etl/queue.h"
#include "etl/list.h"

#include <rcl/rcl.h>
#include <rclc/executor.h>
#include <rclc/rclc.h>
#include <rmw_microros/rmw_microros.h>
#include <uxr/client/transport.h>

#include "BSP_MicroROS.hpp"
#include "Bus/UART_Base.hpp"

#ifndef MICROROS_BUF_SIZE
#define MICROROS_BUF_SIZE 2048
#endif

#ifndef MICROROS_DMA_BUF_SIZE
#define MICROROS_DMA_BUF_SIZE 512
#endif

#ifndef MICROROS_NODE_NAME
#define MICROROS_NODE_NAME "FineMote"
#endif

#ifndef MICROROS_MAX_AGENTS
#define MICROROS_MAX_AGENTS 10
#endif

template<bool enable>
class MicroROS_Base {
public:
    enum class State {
        WAITING_AGENT,
        INITIALIZING,
        RUNNING,
        ERROR
    };

    static MicroROS_Base& GetInstance() {
        static MicroROS_Base instance;
        return instance;
    }

    void Init() {
        state_ = State::WAITING_AGENT;
    }

    void RegisterAgent(ROSAgent<>* agent) {
        if (!agents_.full()) {
            agents_.push_back(agent);
        }
    }

    etl::list<ROSAgent<>*, MICROROS_MAX_AGENTS>& GetAgents() {
        return agents_;
    }

    void Handle() {
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
                osDelay(1000);
                state_ = State::WAITING_AGENT;
                break;
        }
    }

    rcl_node_t* GetNode() { return &node_; }
    rclc_support_t* GetSupport() { return &support_; }
    bool IsRunning() const { return state_ == State::RUNNING; }

private:

    MicroROS_Base()
        : state_(State::WAITING_AGENT),
        last_tick_(0),
        dma_buffer_([this](uint8_t* data, size_t size) { this->PushRxData(data, size); })
    {
        setup();
    }

    void setup() {
        allocator_ = rcl_get_default_allocator();

        rx_sem_ = osSemaphoreNew(1, 0, nullptr);
        tx_sem_ = osSemaphoreNew(1, 1, nullptr);

        UART_Base<5>::GetInstance().BindTxHandle([this]() {
            osSemaphoreRelease(this->tx_sem_);
            return true;
        });

        rmw_uros_set_custom_transport(
            true,
            nullptr,
            TransportOpen,
            TransportClose,
            TransportWrite,
            TransportRead
        );
    }

    ~MicroROS_Base() = default;

    void PushRxData(uint8_t* data, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            if (!rx_queue_.full()) {
                rx_queue_.push(data[i]);
            }
        }
        osSemaphoreRelease(rx_sem_);
    }

    void HandleWaiting() {
        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            state_ = State::INITIALIZING;
        }
    }

    void HandleInitializing() {
        rcl_ret_t ret;

        ret = rclc_support_init(&support_, 0, nullptr, &allocator_);
        if (ret != RCL_RET_OK) { GotoError(); return; }

        ret = rclc_node_init_default(&node_, MICROROS_NODE_NAME, "", &support_);
        if (ret != RCL_RET_OK) { GotoError(); return; }

        size_t handle_count = 0;
        for (auto* agent : agents_) {
            if (!agent->Init(&node_, &support_)) {
                GotoError();
                return;
            }
            handle_count += agent->GetHandleCount();
        }

        handle_count = (handle_count > 0) ? handle_count : 1;
        ret = rclc_executor_init(&executor_, &support_.context, handle_count, &allocator_);
        if (ret != RCL_RET_OK) { GotoError(); return; }

        for (auto* agent : agents_) {
            if (!agent->AddToExecutor(&executor_)) {
                GotoError();
                return;
            }
        }

        last_tick_ = xTaskGetTickCount();
        state_ = State::RUNNING;
    }

    void HandleRunning() {
        rcl_ret_t ret = rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));

        if (ret != RCL_RET_OK && ret != RCL_RET_TIMEOUT) {
            GotoError();
            return;
        }

        for (auto* agent : agents_) {
            agent->Execute();
        }

        if ((xTaskGetTickCount() - last_tick_) > pdMS_TO_TICKS(1000)) {
            last_tick_ = xTaskGetTickCount();
            if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) {
                GotoError();
            }
        }
    }

    void GotoError() {
        Cleanup();
        state_ = State::ERROR;
    }

    void Cleanup() {
        for (auto* agent : agents_) {
            agent->Reset();
        }
        rclc_executor_fini(&executor_);
        rcl_node_fini(&node_);
        rclc_support_fini(&support_);
    }

    static bool TransportOpen(struct uxrCustomTransport* t) {
        auto& self = GetInstance();
        while (!self.rx_queue_.empty()) {
            self.rx_queue_.pop();
        }
        return true;
    }

    static bool TransportClose(struct uxrCustomTransport* t) {
        return true;
    }

    static size_t TransportWrite(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err) {
        auto& self = GetInstance();
        auto& uart = UART_Base<5>::GetInstance();

        if (len > MICROROS_BUF_SIZE) {
            len = MICROROS_BUF_SIZE;
        }

        if (osSemaphoreAcquire(self.tx_sem_, 100) == osOK) {
            memcpy(self.tx_buffer_, buf, len);

            uart.Transmit(self.tx_buffer_, static_cast<uint16_t>(len));

            return len;
        }

        return 0;
    }

    static size_t TransportRead(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
        auto& self = GetInstance();
        size_t read_count = 0;

        uint32_t timeout_ms = (timeout <= 0) ? 0 : static_cast<uint32_t>(timeout);
        uint32_t start_tick = osKernelGetTickCount();

        while (read_count < len) {
            while (read_count < len && !self.rx_queue_.empty()) {
                buf[read_count++] = self.rx_queue_.front();
                self.rx_queue_.pop();
            }

            if (read_count >= len || timeout_ms == 0) {
                break;
            }

            uint32_t elapsed = osKernelGetTickCount() - start_tick;
            if (elapsed >= timeout_ms) {
                break;
            }

            osSemaphoreAcquire(self.rx_sem_, timeout_ms - elapsed);
        }

        return read_count;
    }

    State state_;
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;
    uint32_t last_tick_;

    osSemaphoreId_t rx_sem_;
    osSemaphoreId_t tx_sem_;

    etl::queue<uint8_t, MICROROS_BUF_SIZE> rx_queue_;
    etl::list<ROSAgent<>*, MICROROS_MAX_AGENTS> agents_;

    uint8_t tx_buffer_[MICROROS_BUF_SIZE];
    UARTBuffer<5, MICROROS_DMA_BUF_SIZE> dma_buffer_;
};

#endif