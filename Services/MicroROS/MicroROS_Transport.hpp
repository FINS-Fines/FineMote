/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_TRANSPORT_HPP
#define FINEMOTE_MICROROS_TRANSPORT_HPP

#include <cstring>
#include <atomic>
#include "Board.h"
#include <uxr/client/transport.h>
#include "Bus/UART_Base.hpp"

constexpr size_t MICROROS_BUF_SIZE = 2048;
constexpr size_t MICROROS_DMA_BUF_SIZE = 512;

class MicroROS_Transport
{
public:
    static MicroROS_Transport& GetInstance()
    {
        static MicroROS_Transport instance;
        return instance;
    }

    static bool Open(struct uxrCustomTransport* t)
    {
        GetInstance().ResetRx();
        return true;
    }

    static bool Close(struct uxrCustomTransport* t)
    {
        return true;
    }

    static size_t Write(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err)
    {
        auto& self = GetInstance();

        if (len > MICROROS_BUF_SIZE)
        {
            len = MICROROS_BUF_SIZE;
        }

        for (int i = 0; i < 100; ++i)
        {
            if (self.TryTransmit(buf, len))
            {
                return len;
            }
            usleep(1000);
        }

        return 0;
    }

    static size_t Read(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err)
    {
        auto& self = GetInstance();
        int remain = timeout;
        do {
            size_t n = self.ReadRxData(buf, len);
            if (n > 0) return n;
            if (timeout <= 0) break;
            usleep(5000);
            remain -= 5;
        } while (remain > 0);
        return 0;
    }

private:
    MicroROS_Transport() :
        dma_buffer_([this](uint8_t* data, size_t size)
        {
            this->PushRxData(data, size);
        })
    {
        UART_Base<MICRO_ROS_UART_ID>::GetInstance().BindTxHandle([this]()
        {
            this->tx_busy_.store(false, std::memory_order_release);
            return true;
        });
    }

    ~MicroROS_Transport() = default;

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

    void ResetRx()
    {
        rx_r_.store(rx_w_.load(std::memory_order_acquire), std::memory_order_release);
    }

    bool TryTransmit(const uint8_t* buf, size_t len)
    {
        bool expected = false;
        if (!tx_busy_.compare_exchange_strong(expected, true, std::memory_order_acquire))
            return false;
        memcpy(tx_buffer_, buf, len);
        UART_Base<MICRO_ROS_UART_ID>::GetInstance().Transmit(tx_buffer_, static_cast<uint16_t>(len));
        return true;
    }

    size_t ReadRxData(uint8_t* buf, size_t len)
    {
        size_t r = rx_r_.load(std::memory_order_relaxed);
        size_t w = rx_w_.load(std::memory_order_acquire);
        size_t avail = w - r;
        if (avail == 0) return 0;
        size_t n = avail < len ? avail : len;
        size_t pos = r & (MICROROS_BUF_SIZE - 1);
        size_t first = MICROROS_BUF_SIZE - pos;
        if (n <= first) {
            memcpy(buf, &rx_buf_[pos], n);
        } else {
            memcpy(buf, &rx_buf_[pos], first);
            memcpy(buf + first, &rx_buf_[0], n - first);
        }
        rx_r_.store(r + n, std::memory_order_release);
        return n;
    }

    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;
    uint8_t rx_buf_[MICROROS_BUF_SIZE];
    std::atomic<size_t> rx_w_{0};
    std::atomic<size_t> rx_r_{0};
    uint8_t tx_buffer_[MICROROS_BUF_SIZE];
    std::atomic<bool> tx_busy_{false};
};

#endif
