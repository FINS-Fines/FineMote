/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_UART_TRANSPORT_HPP
#define FINEMOTE_MICROROS_UART_TRANSPORT_HPP

#include <atomic>
#include <cstddef>
#include <cstdint>
#include <cstring>

#include "BSP_POSIX.h"

#include <rmw_microros/custom_transport.h>
#include <uxr/client/transport.h>

#include "Bus/UART_Base.hpp"
#include "MicroROS/MicroROS_Backend.hpp"

template<uint8_t UartId, size_t BufferSize = 2048, size_t DmaBufferSize = 512>
class MicroROS_UartTransport final: public MicroROS_Backend {
    static_assert((BufferSize & (BufferSize - 1)) == 0, "BufferSize must be a power of two");

public:
    MicroROS_UartTransport():
        dma_buffer_([this](uint8_t* data, size_t size) {
            PushRxData(data, size);
        }) {
        UART_Base<UartId>::GetInstance().BindTxHandle([this]() {
            tx_busy_.store(false, std::memory_order_release);
            return true;
        });
    }

    bool Prepare() final {
        return true;
    }

    rmw_ret_t Configure(rmw_init_options_t* options) final {
        return rmw_uros_options_set_custom_transport(true, this, Open, Close, Write, Read, options);
    }

private:
    static MicroROS_UartTransport* GetSelf(uxrCustomTransport* transport) {
        return transport == nullptr ? nullptr : static_cast<MicroROS_UartTransport*>(transport->args);
    }

    static bool Open(uxrCustomTransport* transport) {
        auto* self = GetSelf(transport);
        if (self == nullptr) {
            return false;
        }
        self->ResetRx();
        return true;
    }

    static bool Close(uxrCustomTransport*) {
        return true;
    }

    static size_t Write(uxrCustomTransport* transport, const uint8_t* buffer, size_t length, uint8_t* error_code) {
        auto* self = GetSelf(transport);
        if (self == nullptr || length > BufferSize) {
            SetError(error_code);
            return 0;
        }

        for (size_t attempt = 0; attempt < 100; ++attempt) {
            if (self->TryTransmit(buffer, length)) {
                return length;
            }
            usleep(1000);
        }

        SetError(error_code);
        return 0;
    }

    static size_t
    Read(uxrCustomTransport* transport, uint8_t* buffer, size_t length, int timeout_ms, uint8_t* error_code) {
        auto* self = GetSelf(transport);
        if (self == nullptr) {
            SetError(error_code);
            return 0;
        }

        int remaining_ms = timeout_ms;
        do {
            const size_t read_size = self->ReadRxData(buffer, length);
            if (read_size > 0) {
                return read_size;
            }
            if (timeout_ms <= 0) {
                break;
            }
            usleep(5000);
            remaining_ms -= 5;
        } while (remaining_ms > 0);

        return 0;
    }

    static void SetError(uint8_t* error_code) {
        if (error_code != nullptr) {
            *error_code = 1;
        }
    }

    void PushRxData(const uint8_t* data, size_t size) {
        if (size == 0) {
            return;
        }

        const size_t write_index = rx_write_.load(std::memory_order_relaxed);
        const size_t read_index = rx_read_.load(std::memory_order_acquire);
        if (size > BufferSize - (write_index - read_index)) {
            return;
        }

        const size_t position = write_index & (BufferSize - 1);
        const size_t first_size = BufferSize - position;
        if (size <= first_size) {
            memcpy(&rx_buffer_[position], data, size);
        } else {
            memcpy(&rx_buffer_[position], data, first_size);
            memcpy(rx_buffer_, data + first_size, size - first_size);
        }
        rx_write_.store(write_index + size, std::memory_order_release);
    }

    void ResetRx() {
        rx_read_.store(rx_write_.load(std::memory_order_acquire), std::memory_order_release);
    }

    bool TryTransmit(const uint8_t* buffer, size_t length) {
        bool expected = false;
        if (!tx_busy_.compare_exchange_strong(expected, true, std::memory_order_acq_rel)) {
            return false;
        }

        memcpy(tx_buffer_, buffer, length);
        UART_Base<UartId>::GetInstance().Transmit(tx_buffer_, static_cast<uint16_t>(length));
        return true;
    }

    size_t ReadRxData(uint8_t* buffer, size_t length) {
        const size_t read_index = rx_read_.load(std::memory_order_relaxed);
        const size_t write_index = rx_write_.load(std::memory_order_acquire);
        const size_t available = write_index - read_index;
        if (available == 0) {
            return 0;
        }

        const size_t read_size = available < length ? available : length;
        const size_t position = read_index & (BufferSize - 1);
        const size_t first_size = BufferSize - position;
        if (read_size <= first_size) {
            memcpy(buffer, &rx_buffer_[position], read_size);
        } else {
            memcpy(buffer, &rx_buffer_[position], first_size);
            memcpy(buffer + first_size, rx_buffer_, read_size - first_size);
        }
        rx_read_.store(read_index + read_size, std::memory_order_release);
        return read_size;
    }

    UARTBuffer<UartId, DmaBufferSize> dma_buffer_;
    uint8_t rx_buffer_[BufferSize] {};
    std::atomic<size_t> rx_write_ { 0 };
    std::atomic<size_t> rx_read_ { 0 };
    uint8_t tx_buffer_[BufferSize] {};
    std::atomic<bool> tx_busy_ { false };
};

#endif
