/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#ifndef FINEMOTE_BSP_MICROROS_HPP
#define FINEMOTE_BSP_MICROROS_HPP

#include "Board.h"
#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"
#include "etl/queue.h"

#include <rcl/rcl.h>
#include <rclc/rclc.h>
#include <rclc/executor.h>
#include <uxr/client/transport.h>
#include <rmw_microros/rmw_microros.h>

#include "Bus/MicroROS_Base.hpp"
#include "Bus/UART_Base.hpp"

#ifndef MICROROS_RX_BUF_SIZE
#define MICROROS_RX_BUF_SIZE 2048
#endif

#ifndef MICROROS_DMA_BUF_SIZE
#define MICROROS_DMA_BUF_SIZE 512
#endif

#ifndef MICROROS_NODE_NAME
#define MICROROS_NODE_NAME "STM32_FineMote_Node"
#endif

template <uint8_t UART_ID>
class BSP_MicroROS {
public:
    static BSP_MicroROS& GetInstance() {
        static BSP_MicroROS instance;
        return instance;
    }

    void Init() {
        state_ = State::WAITING_AGENT;
    }

    void Spin() {
        switch (state_) {
            case State::WAITING_AGENT:
                HandleWaiting();
                break;
            case State::INITIALIZING:
                HandleInit();
                break;
            case State::RUNNING:
                HandleRunning();
                break;
            case State::ERROR_RECOVERY:
                HandleError();
                break;
        }
    }

    rcl_node_t* GetNode() { return &node_; }
    rclc_support_t* GetSupport() { return &support_; }
    bool IsRunning() const { return state_ == State::RUNNING; }

private:
    BSP_MicroROS()
        : state_(State::WAITING_AGENT),
          last_tick_(0),
          dmaBuffer_([this](uint8_t* data, size_t size) {
              this->PushRxData(data, size);
          })
    {
        allocator_ = rcl_get_default_allocator();
    }

    ~BSP_MicroROS() = default;

    enum class State {
        WAITING_AGENT,
        INITIALIZING,
        RUNNING,
        ERROR_RECOVERY
    };

    void PushRxData(uint8_t* data, size_t size) {
        for (size_t i = 0; i < size; ++i) {
            if (!rx_queue_.full()) {
                rx_queue_.push(data[i]);
            }
        }
    }

    void HandleWaiting() {
        rmw_uros_set_custom_transport(
            true, nullptr,
            TransportOpen, TransportClose, TransportWrite, TransportRead
        );

        if (rmw_uros_ping_agent(100, 1) == RMW_RET_OK) {
            state_ = State::INITIALIZING;
        }
    }

    void HandleInit() {
        rcl_ret_t rc;

        rc = rclc_support_init(&support_, 0, nullptr, &allocator_);
        if (rc != RCL_RET_OK) { state_ = State::ERROR_RECOVERY; return; }

        rc = rclc_node_init_default(&node_, MICROROS_NODE_NAME, "", &support_);
        if (rc != RCL_RET_OK) { state_ = State::ERROR_RECOVERY; return; }

        size_t handle_count = 0;
        for (ROSAgent* agent = ROSAgent::GetHead(); agent != nullptr; agent = agent->GetNext()) {
            if (!agent->Init(&node_, &support_)) {
                state_ = State::ERROR_RECOVERY;
                return;
            }
            handle_count += agent->GetHandleCount();
        }

        handle_count = (handle_count > 0) ? handle_count : 1;
        rc = rclc_executor_init(&executor_, &support_.context, handle_count, &allocator_);
        if (rc != RCL_RET_OK) { state_ = State::ERROR_RECOVERY; return; }

        for (ROSAgent* agent = ROSAgent::GetHead(); agent != nullptr; agent = agent->GetNext()) {
            if (!agent->AddToExecutor(&executor_)) {
                 state_ = State::ERROR_RECOVERY;
                 return;
            }
        }

        last_tick_ = xTaskGetTickCount();
        state_ = State::RUNNING;
    }

    void HandleRunning() {
        rclc_executor_spin_some(&executor_, RCL_MS_TO_NS(1));

        if (xTaskGetTickCount() - last_tick_ > 1000) {
            last_tick_ = xTaskGetTickCount();
            if (rmw_uros_ping_agent(10, 1) != RMW_RET_OK) {
                state_ = State::ERROR_RECOVERY;
            }
        }
    }

    void HandleError() {
        for (ROSAgent* agent = ROSAgent::GetHead(); agent != nullptr; agent = agent->GetNext()) {
            agent->Reset();
        }
        rclc_executor_fini(&executor_);
        rcl_node_fini(&node_);
        rclc_support_fini(&support_);

        state_ = State::WAITING_AGENT;
    }

    static bool TransportOpen(struct uxrCustomTransport* t) {
        auto& self = GetInstance();
        taskENTER_CRITICAL();
        while(!self.rx_queue_.empty()) {
            self.rx_queue_.pop();
        }
        taskEXIT_CRITICAL();
        return true;
    }

    static bool TransportClose(struct uxrCustomTransport* t) {
        return true;
    }

    static size_t TransportWrite(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err) {
        UART_HandleTypeDef* huart = BSP_UARTList[UART_ID];

        if (huart == nullptr) return 0;

        HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)buf, len, 10);
        return (status == HAL_OK) ? len : 0;
    }

    static size_t TransportRead(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err) {
        auto& self = GetInstance();
        size_t read_count = 0;

        TickType_t timeout_ticks = (timeout > 0) ? pdMS_TO_TICKS(timeout) : 0;
        TickType_t start_tick = xTaskGetTickCount();

        do {
            taskENTER_CRITICAL();
            while (read_count < len && !self.rx_queue_.empty()) {
                buf[read_count++] = self.rx_queue_.front();
                self.rx_queue_.pop();
            }
            taskEXIT_CRITICAL();

            if (read_count >= len || timeout == 0) {
                break;
            }

            if ((xTaskGetTickCount() - start_tick) < timeout_ticks) {
                 osDelay(1);
            }

        } while ((xTaskGetTickCount() - start_tick) < timeout_ticks);

        return read_count;
    }

    State state_;
    rcl_allocator_t allocator_;
    rclc_support_t support_;
    rcl_node_t node_;
    rclc_executor_t executor_;
    uint32_t last_tick_;

    etl::queue<uint8_t, MICROROS_RX_BUF_SIZE> rx_queue_;

    UARTBuffer<UART_ID, MICROROS_DMA_BUF_SIZE> dmaBuffer_;
};

#endif // FINEMOTE_BSP_MICROROS_HPP