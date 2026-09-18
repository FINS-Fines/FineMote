/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_PCF8574_HPP
#define FINEMOTE_PCF8574_HPP

#include <cstddef>
#include <cstdint>

#include "Bus/I2C_Base.hpp"
#include "DeviceBase/DeviceBase.hpp"
#include "esp_log.h"

template<size_t BusId>
class PCF8574 final: public DeviceBase {
public:
    using SampleCallback = void (*)(void* context, PlatformErr result, uint8_t value);

    static constexpr uint16_t DefaultAddress = 0x20U;
    static constexpr uint32_t DefaultClockSpeedHz = 100000U;

    explicit PCF8574(const I2C_DeviceConfig& config, uint32_t divisionFactor = 1U);

    explicit PCF8574(
        uint16_t address = DefaultAddress,
        uint32_t clockSpeedHz = DefaultClockSpeedHz,
        uint32_t divisionFactor = 1U
    );

    PCF8574(const PCF8574&) = delete;
    PCF8574& operator=(const PCF8574&) = delete;

    PlatformErr Write(uint8_t value);

    PlatformErr Read(uint8_t& value);

    PlatformErr ReleaseInputs();

    [[nodiscard]] uint8_t LastInput() const;

    [[nodiscard]] uint8_t OutputLatch() const;

    [[nodiscard]] PlatformErr LastResult() const;

    void SetSampleCallback(SampleCallback callback, void* context = nullptr);

    void SetFailureRetryCycles(uint32_t cycles);

    void Update() final;

    void Handle() final;

private:
    static constexpr const char* Tag = "PCF8574";

    I2C_Agent<BusId> i2cAgent_;
    uint8_t outputLatch_ = 0xFFU;
    uint8_t lastInput_ = 0U;
    PlatformErr lastResult_ = PLATFORM_NOT_INITIALIZED;
    SampleCallback sampleCallback_ = nullptr;
    void* sampleContext_ = nullptr;
    uint8_t scheduledValue_ = 0U;
    PlatformErr scheduledResult_ = PLATFORM_NOT_INITIALIZED;
    uint32_t failureRetryCycles_ = 0U;
    uint32_t retryCyclesRemaining_ = 0U;
    bool samplePending_ = false;
};

// Template method implementations

template<size_t BusId>
PCF8574<BusId>::PCF8574(const I2C_DeviceConfig& config, uint32_t divisionFactor):
    DeviceBase(divisionFactor),
    i2cAgent_(config) {
    ESP_EARLY_LOGI(
        Tag,
        "Initialized on I2C%u, address=0x%02X",
        static_cast<unsigned>(BusId),
        static_cast<unsigned>(config.deviceAddress)
    );
}

template<size_t BusId>
PCF8574<BusId>::PCF8574(uint16_t address, uint32_t clockSpeedHz, uint32_t divisionFactor):
    PCF8574(
        I2C_DeviceConfig {
            I2C_DeviceAddressWidth::Bits7,
            address,
            clockSpeedHz,
        },
        divisionFactor
    ) {}

template<size_t BusId>
PlatformErr PCF8574<BusId>::Write(uint8_t value) {
    const PlatformErr result = i2cAgent_.Transmit(&value, 1U);
    lastResult_ = result;
    if (result == PLATFORM_OK) {
        outputLatch_ = value;
        ESP_LOGD(Tag, "Write value=0x%02X", static_cast<unsigned>(value));
    } else {
        ESP_LOGE(Tag, "Write failed: err=%d", static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr PCF8574<BusId>::Read(uint8_t& value) {
    const PlatformErr result = i2cAgent_.Receive(&value, 1U);
    lastResult_ = result;
    if (result == PLATFORM_OK) {
        lastInput_ = value;
        ESP_LOGD(Tag, "Read value=0x%02X", static_cast<unsigned>(value));
    } else {
        ESP_LOGE(Tag, "Read failed: err=%d", static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr PCF8574<BusId>::ReleaseInputs() {
    return Write(0xFFU);
}

template<size_t BusId>
[[nodiscard]] uint8_t PCF8574<BusId>::LastInput() const {
    return lastInput_;
}

template<size_t BusId>
[[nodiscard]] uint8_t PCF8574<BusId>::OutputLatch() const {
    return outputLatch_;
}

template<size_t BusId>
[[nodiscard]] PlatformErr PCF8574<BusId>::LastResult() const {
    return lastResult_;
}

template<size_t BusId>
void PCF8574<BusId>::SetSampleCallback(typename PCF8574<BusId>::SampleCallback callback, void* context) {
    sampleCallback_ = callback;
    sampleContext_ = context;
}

template<size_t BusId>
void PCF8574<BusId>::SetFailureRetryCycles(uint32_t cycles) {
    failureRetryCycles_ = cycles;
}

template<size_t BusId>
void PCF8574<BusId>::Update() {
    samplePending_ = false;
    if (sampleCallback_ == nullptr)
        return;
    if (retryCyclesRemaining_ > 0U) {
        --retryCyclesRemaining_;
        return;
    }

    scheduledResult_ = Read(scheduledValue_);
    samplePending_ = true;
    if (scheduledResult_ != PLATFORM_OK) {
        retryCyclesRemaining_ = failureRetryCycles_;
    }
}

template<size_t BusId>
void PCF8574<BusId>::Handle() {
    if (!samplePending_ || sampleCallback_ == nullptr)
        return;
    sampleCallback_(sampleContext_, scheduledResult_, scheduledValue_);
    samplePending_ = false;
}

#endif // FINEMOTE_PCF8574_HPP
