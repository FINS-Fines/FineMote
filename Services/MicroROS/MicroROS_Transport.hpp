#ifndef FINEMOTE_MICROROS_TRANSPORT_HPP
#define FINEMOTE_MICROROS_TRANSPORT_HPP

#include <cstring>
#include "Board.h"
#include "cmsis_os.h"
#include <uxr/client/transport.h>
#include "Bus/UART_Base.hpp"

#ifndef MICROROS_BUF_SIZE
#define MICROROS_BUF_SIZE 2048
#endif

#ifndef MICROROS_DMA_BUF_SIZE
#define MICROROS_DMA_BUF_SIZE 512
#endif

template <bool enable>
class MicroROS_Manager;

template <bool enable>
class MicroROS_Transport
{
public:
    static bool Open(struct uxrCustomTransport* t)
    {
        auto& manager = MicroROS_Manager<enable>::GetInstance();
        while (!manager.rx_queue_.empty())
        {
            manager.rx_queue_.pop();
        }
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

        if (osSemaphoreAcquire(manager.tx_sem_, 100) == osOK)
        {
            memcpy(manager.tx_buffer_, buf, len);
            UART_Base<MICRO_ROS_UART_ID>::GetInstance().Transmit(
                manager.tx_buffer_,
                static_cast<uint16_t>(len)
            );
            return len;
        }

        return 0;
    }

    static size_t Read(struct uxrCustomTransport* t, uint8_t* buf, size_t len, int timeout, uint8_t* err)
    {
        auto& manager = MicroROS_Manager<enable>::GetInstance();
        size_t read_count = 0;

        uint32_t timeout_ms = (timeout <= 0) ? 0 : static_cast<uint32_t>(timeout);
        uint32_t start_tick = osKernelGetTickCount();

        while (read_count < len)
        {
            while (read_count < len && !manager.rx_queue_.empty())
            {
                buf[read_count++] = manager.rx_queue_.front();
                manager.rx_queue_.pop();
            }

            if (read_count >= len || timeout_ms == 0)
            {
                break;
            }

            uint32_t elapsed = osKernelGetTickCount() - start_tick;
            if (elapsed >= timeout_ms)
            {
                break;
            }

            osSemaphoreAcquire(manager.rx_sem_, timeout_ms - elapsed);
        }

        return read_count;
    }
};

#endif
