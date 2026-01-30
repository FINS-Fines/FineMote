/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_PORT_HPP
#define FINEMOTE_MICROROS_PORT_HPP

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h"

#include "Bus/UART_Base.hpp"
#include "MC_Board.h"
#include "etl/queue.h"
#include <uxr/client/transport.h>

#define MICROROS_RX_BUFFER_SIZE 2048
#define DMA_BLOCK_SIZE 512

/**
 * @brief Micro-ROS 硬件传输适配器
 * 适配 UART_Base 框架与 Micro-ROS XRCE-DDS 中间件
 *
 * @tparam UART_ID 串口 ID (对应 MC_Board.h 中的索引)
 */
template<uint8_t UART_ID>
class MicroROSPort {
public:

    static MicroROSPort &GetInstance() {
        static MicroROSPort instance;
        return instance;
    }

    // --- Micro-ROS C 接口适配 ---

    static bool TransportOpen(struct uxrCustomTransport *transport) {
        MicroROSPort& instance = GetInstance();

        // taskENTER_CRITICAL();
        while(!instance.rxQueue.empty()) {
            instance.rxQueue.pop();
        }
        // taskEXIT_CRITICAL();

        return true;
    }

    static bool TransportClose(struct uxrCustomTransport *transport) {
        // 通常不需要关闭串口硬件
        return true;
    }

        static size_t TransportWrite(struct uxrCustomTransport *transport, const uint8_t *buf, size_t len, uint8_t *err) {
        // 为了稳定性，直接调用 HAL 库的阻塞发送。

        UART_HandleTypeDef* huart = BSP_UARTList[UART_ID];
        if (huart == nullptr) {
            return 0;
        }

        // 使用阻塞模式发送，超时时间设为 10ms (根据波特率调整)
        HAL_StatusTypeDef status = HAL_UART_Transmit(huart, (uint8_t*)buf, len, 10);

        return (status == HAL_OK) ? len : 0;
    }

    static size_t TransportRead(struct uxrCustomTransport *transport, uint8_t *buf, size_t len, int timeout, uint8_t *err) {
        MicroROSPort& instance = GetInstance();
        size_t read_count = 0;

        // 简单的超时轮询逻辑
        TickType_t start_tick = xTaskGetTickCount();
        TickType_t timeout_ticks = pdMS_TO_TICKS(timeout);

        do {
            // 临界区保护：防止 pop 时被 ISR push 打断
            taskENTER_CRITICAL();
            while (read_count < len && !instance.rxQueue.empty()) {
                buf[read_count++] = instance.rxQueue.front();
                instance.rxQueue.pop();
            }
            taskEXIT_CRITICAL();

            if (read_count >= len) break;

            if (timeout > 0) {
                osDelay(1);
            }

        } while ((xTaskGetTickCount() - start_tick) < timeout_ticks);

        return read_count;
    }

private:
    // 构造函数：初始化 Buffer 并绑定到 UART_Base
    MicroROSPort() : dmaBuffer([this](uint8_t *data, size_t size) {
        this->OnRxData(data, size);
    }) {
        // 确保 UART_Base 引用了我们的 dmaBuffer
        // UARTBuffer 的构造函数会自动调用 Bind
    }

    // 回调函数：由 UART_Base 在 ISR 上下文中调用
    void OnRxData(uint8_t *data, size_t size) {
        // 注意：这是在中断上下文中运行的
        // 将数据压入环形缓冲
        for (size_t i = 0; i < size; ++i) {
            if (!rxQueue.full()) {
                rxQueue.push(data[i]);
            } else {
                // 缓冲区满，丢包（或者可以在这里统计丢包率）
                break;
            }
        }
    }

    // 成员变量
    // 用于接收底层 DMA 数据的双缓冲
    UARTBuffer<UART_ID, DMA_BLOCK_SIZE> dmaBuffer;

    // 用于给 Micro-ROS 读取的环形缓冲 (FIFO)
    etl::queue<uint8_t, MICROROS_RX_BUFFER_SIZE> rxQueue;
};

#endif //FINEMOTE_MICROROS_PORT_HPP
