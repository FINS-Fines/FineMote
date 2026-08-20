/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_USB_TRANSPORT_HPP
#define FINEMOTE_MICROROS_USB_TRANSPORT_HPP

#include <cstddef>
#include <cstdint>
#include <cstring>

#include "FreeRTOS.h"
#include "task.h"
#include "usb_device.h"
#include "usbd_cdc.h"
#include "usbd_cdc_if.h"

#include <rmw_microros/custom_transport.h>
#include <uxr/client/transport.h>
#include <uxr/client/util/time.h>

#include "MicroROS/MicroROS_Backend.hpp"

extern USBD_HandleTypeDef hUsbDeviceHS;
//todo：有必要的话再用模板重写这个

class MicroROS_USB_Transport final : public MicroROS_Backend {
    static constexpr size_t BufferSize = 2048;
    static constexpr uint32_t WriteTimeoutMs = 100;

public:
    bool Prepare() final {
        taskENTER_CRITICAL();
        active_ = this;
        rx_head_ = rx_tail_;
        if (!initialized_) {
            USBD_Interface_fops_HS.Control = Control;
            USBD_Interface_fops_HS.Receive = Receive;
            USBD_Interface_fops_HS.TransmitCplt = TransmitComplete;
            initialized_ = true;
        }
        taskEXIT_CRITICAL();
        return true;
    }

    rmw_ret_t Configure(rmw_init_options_t* options) final {
        return rmw_uros_options_set_custom_transport(true, this, Open, Close, Write, Read, options);
    }

private:
    static MicroROS_USB_Transport* Self(uxrCustomTransport* transport) {
        return transport == nullptr ? nullptr : static_cast<MicroROS_USB_Transport*>(transport->args);
    }

    static bool Open(uxrCustomTransport* transport) {
        auto* self = Self(transport);
        if (self == nullptr) {
            return false;
        }
        taskENTER_CRITICAL();
        active_ = self;
        self->rx_head_ = self->rx_tail_;
        taskEXIT_CRITICAL();
        return true;
    }

    static bool Close(uxrCustomTransport*) {
        return true;
    }

    static size_t Write(uxrCustomTransport* transport, const uint8_t* buffer,
                        size_t length, uint8_t* error_code) {
        SetError(error_code, 0);
        auto* self = Self(transport);
        if (self == nullptr || buffer == nullptr || length == 0 || length > UINT16_MAX ||
            hUsbDeviceHS.dev_state != USBD_STATE_CONFIGURED) {
            SetError(error_code, 1);
            return 0;
        }

        self->write_complete_ = false;
        if (CDC_Transmit_HS(const_cast<uint8_t*>(buffer), static_cast<uint16_t>(length)) != USBD_OK) {
            SetError(error_code, 1);
            return 0;
        }

        const int64_t start = uxr_millis();
        while (!self->write_complete_ && (uxr_millis() - start) < WriteTimeoutMs) {
            vTaskDelay(pdMS_TO_TICKS(1));
        }
        if (!self->write_complete_) {
            SetError(error_code, 1);
            return 0;
        }
        return length;
    }

    static size_t Read(uxrCustomTransport* transport, uint8_t* buffer,
                       size_t length, int timeout, uint8_t* error_code) {
        SetError(error_code, 0);
        auto* self = Self(transport);
        if (self == nullptr || buffer == nullptr || length == 0) {
            SetError(error_code, 1);
            return 0;
        }

        const int64_t start = uxr_millis();
        do {
            const uint32_t head = self->rx_head_;
            const uint32_t tail = self->rx_tail_;
            const uint32_t available = tail - head;
            if (available != 0) {
                const size_t read_length = available < length ? available : length;
                const size_t position = head & (BufferSize - 1);
                const size_t first = BufferSize - position;
                if (read_length <= first) {
                    memcpy(buffer, &self->rx_buffer_[position], read_length);
                } else {
                    memcpy(buffer, &self->rx_buffer_[position], first);
                    memcpy(buffer + first, self->rx_buffer_, read_length - first);
                }
                __DMB();
                self->rx_head_ = head + static_cast<uint32_t>(read_length);
                return read_length;
            }
            if (timeout <= 0) {
                break;
            }
            vTaskDelay(pdMS_TO_TICKS(1));
        } while ((uxr_millis() - start) < timeout);
        return 0;
    }

    static int8_t Control(uint8_t command, uint8_t* buffer, uint16_t length) {
        static uint8_t line_coding[7] = {0x00, 0xC2, 0x01, 0x00, 0x00, 0x00, 0x08};
        switch (command) {
            case CDC_SET_LINE_CODING:
                if (buffer != nullptr && length >= sizeof(line_coding)) {
                    memcpy(line_coding, buffer, sizeof(line_coding));
                }
                break;
            case CDC_GET_LINE_CODING:
                if (buffer != nullptr && length >= sizeof(line_coding)) {
                    memcpy(buffer, line_coding, sizeof(line_coding));
                }
                break;
            default:
                break;
        }
        return static_cast<int8_t>(USBD_OK);
    }

    static int8_t Receive(uint8_t* buffer, uint32_t* length) {
        auto* self = active_;
        if (self != nullptr && buffer != nullptr && length != nullptr) {
            const uint32_t received = *length;
            const uint32_t tail = self->rx_tail_;
            const uint32_t head = self->rx_head_;
            const uint32_t free_space = BufferSize - (tail - head);
            if (received <= free_space) {
                const size_t position = tail & (BufferSize - 1);
                const size_t first = BufferSize - position;
                if (received <= first) {
                    memcpy(&self->rx_buffer_[position], buffer, received);
                } else {
                    memcpy(&self->rx_buffer_[position], buffer, first);
                    memcpy(self->rx_buffer_, buffer + first, received - first);
                }
                __DMB();
                self->rx_tail_ = tail + received;
            }
        }
        USBD_CDC_SetRxBuffer(&hUsbDeviceHS, buffer);
        USBD_CDC_ReceivePacket(&hUsbDeviceHS);
        return static_cast<int8_t>(USBD_OK);
    }

    static int8_t TransmitComplete(uint8_t*, uint32_t*, uint8_t) {
        if (active_ != nullptr) {
            active_->write_complete_ = true;
        }
        return static_cast<int8_t>(USBD_OK);
    }

    static void SetError(uint8_t* error_code, uint8_t value) {
        if (error_code != nullptr) {
            *error_code = value;
        }
    }

    static inline MicroROS_USB_Transport* active_ = nullptr;
    static inline bool initialized_ = false;
    volatile uint32_t rx_head_ = 0;
    volatile uint32_t rx_tail_ = 0;
    volatile bool write_complete_ = false;
    uint8_t rx_buffer_[BufferSize] {};
};

#endif
