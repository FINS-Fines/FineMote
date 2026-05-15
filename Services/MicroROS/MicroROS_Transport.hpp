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

template <bool enable>
class MicroROS_Manager;

template <bool enable>
class MicroROS_Transport
{
public:
    static bool Open(struct uxrCustomTransport* t)
    {
        auto& m = MicroROS_Manager<enable>::GetInstance();
        m.rx_r_.store(m.rx_w_.load(std::memory_order_acquire), std::memory_order_release);
        return true;
    }

    static bool Close(struct uxrCustomTransport* t)
    {
        return true;
    }

    static size_t Write(struct uxrCustomTransport* t, const uint8_t* buf, size_t len, uint8_t* err)
    {
        auto& manager = MicroROS_Manager<enable>::GetInstance();

        if (len > MICROROS_BUF_SIZE)
        {
            len = MICROROS_BUF_SIZE;
        }

        for (int i = 0; i < 100; ++i)
        {
            bool expected = false;
            if (manager.tx_busy_.compare_exchange_strong(expected, true, std::memory_order_acquire))
            {
                memcpy(manager.tx_buffer_, buf, len);
                UART_Base<MICRO_ROS_UART_ID>::GetInstance().Transmit(
                    manager.tx_buffer_,
                    static_cast<uint16_t>(len)
                );
                return len;
            }
            usleep(1000);
        }

        return 0;
    }

    static size_t Read(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err)
    {
        auto& m = MicroROS_Manager<enable>::GetInstance();
        int remain = timeout;
        do {
            size_t r = m.rx_r_.load(std::memory_order_relaxed);
            size_t w = m.rx_w_.load(std::memory_order_acquire);
            size_t avail = w - r;
            if (avail > 0) {
                size_t n = avail < len ? avail : len;
                size_t pos = r & (MICROROS_BUF_SIZE - 1);
                size_t first = MICROROS_BUF_SIZE - pos;
                if (n <= first) {
                    memcpy(buf, &m.rx_buf_[pos], n);
                } else {
                    memcpy(buf, &m.rx_buf_[pos], first);
                    memcpy(buf + first, &m.rx_buf_[0], n - first);
                }
                m.rx_r_.store(r + n, std::memory_order_release);
                return n;
            }
            if (timeout <= 0) break;
            usleep(5000);
            remain -= 5;
        } while (remain > 0);
        return 0;
    }
};

#endif
