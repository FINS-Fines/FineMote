#ifndef FINEMOTE_MICROROS_TRANSPORT_HPP
#define FINEMOTE_MICROROS_TRANSPORT_HPP

#include <cstring>
#include "Board.h"
#include "cmsis_os.h"
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
        auto& manager = MicroROS_Manager<enable>::GetInstance();
        xStreamBufferReset(manager.rx_stream_buffer_);
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

        if (xSemaphoreTake(manager.tx_sem_, pdMS_TO_TICKS(100)) == pdTRUE)
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
        uint32_t timeout_ticks = (timeout <= 0) ? 0 : pdMS_TO_TICKS(timeout);

        return xStreamBufferReceive(
            manager.rx_stream_buffer_,
            buf,
            len,
            timeout_ticks
        );
    }
};

#endif
