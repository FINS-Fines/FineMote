/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_ADS1115_HPP
#define FINEMOTE_ADS1115_HPP

#include <cstddef>
#include <cstdint>

#include "Bus/I2C_Base.hpp"
#include "DeviceBase/DeviceBase.hpp"
#include "driver/gpio.h"
#include "esp_err.h"
#include "esp_log.h"
#include "esp_rom_sys.h"

struct ADS1115_Config {
    I2C_DeviceConfig i2c {
        I2C_DeviceAddressWidth::Bits7,
        0x48U,
        100000U,
    };
    float dividerRatio = 12.0F;
    float currentSenseResistanceOhm = 0.0003F;
    float currentSenseGain = 100.0F;
    gpio_num_t readyPin = GPIO_NUM_4;
    uint32_t conversionTimeoutUs = 10000U;
    uint32_t conversionPollIntervalUs = 200U;
};

struct ADS1115_Values {
    float input1Voltage = 0.0F;
    float input2Voltage = 0.0F;
    float outputVoltage = 0.0F;
    float outputCurrent = 0.0F;
};

template<size_t BusId>
class ADS1115 final: public DeviceBase {
public:
    using SampleCallback = void (*)(void* context, PlatformErr result, const ADS1115_Values& values);

    explicit ADS1115(const ADS1115_Config& config = {}, uint32_t divisionFactor = 1U);

    ADS1115(const ADS1115&) = delete;
    ADS1115& operator=(const ADS1115&) = delete;

    PlatformErr ReadRaw(uint8_t channel, int16_t& value);

    PlatformErr ReadAll(ADS1115_Values& values);

    [[nodiscard]] const ADS1115_Values& Values() const;

    [[nodiscard]] PlatformErr GpioInitResult() const;

    [[nodiscard]] PlatformErr LastResult() const;

    void SetSampleCallback(SampleCallback callback, void* context = nullptr);

    void SetFailureRetryCycles(uint32_t cycles);

    void Update() final;

    void Handle() final;

private:
    static constexpr const char* Tag = "ADS1115";

    static constexpr uint8_t ChannelCount = 4U;
    static constexpr uint16_t ConversionRegister = 0x00U;
    static constexpr uint16_t ConfigRegister = 0x01U;
    static constexpr uint16_t ConfigStartSingleConversion = 0x8000U;
    static constexpr uint16_t SingleEndedMuxBase = 0x04U;
    static constexpr uint8_t MuxShift = 12U;
    static constexpr uint16_t ConfigGainTwoVolts = 0x0400U;
    static constexpr uint16_t ConfigSingleShot = 0x0100U;
    static constexpr uint16_t ConfigDataRate860Sps = 0x00E0U;
    static constexpr uint16_t ConfigDisableComparator = 0x0003U;
    static constexpr uint16_t ConfigConversionReady = 0x8000U;
    static constexpr float VoltsPerLsb = 62.5e-6F;

    PlatformErr WriteRegister(uint16_t reg, uint16_t value);

    PlatformErr ReadRegister(uint16_t reg, uint16_t& value);

    PlatformErr WaitForConversion();

    ADS1115_Config config_;
    I2C_Agent<BusId> i2cAgent_;
    ADS1115_Values values_ {};
    PlatformErr gpioInitResult_ = PLATFORM_NOT_INITIALIZED;
    PlatformErr lastResult_ = PLATFORM_NOT_INITIALIZED;
    SampleCallback sampleCallback_ = nullptr;
    void* sampleContext_ = nullptr;
    ADS1115_Values scheduledValues_ {};
    PlatformErr scheduledResult_ = PLATFORM_NOT_INITIALIZED;
    uint32_t failureRetryCycles_ = 0U;
    uint32_t retryCyclesRemaining_ = 0U;
    bool samplePending_ = false;
};

// Template method implementations

template<size_t BusId>
ADS1115<BusId>::ADS1115(const ADS1115_Config& config, uint32_t divisionFactor):
    DeviceBase(divisionFactor),
    config_(config),
    i2cAgent_(config.i2c) {
    if (config_.readyPin < 0) {
        gpioInitResult_ = PLATFORM_INVALID_PARAM;
        ESP_EARLY_LOGE(Tag, "Invalid READY GPIO: %d", static_cast<int>(config_.readyPin));
        return;
    }

    gpio_config_t ioConfig {};
    ioConfig.pin_bit_mask = 1ULL << static_cast<uint32_t>(config_.readyPin);
    ioConfig.mode = GPIO_MODE_INPUT;
    ioConfig.pull_up_en = GPIO_PULLUP_ENABLE;
    ioConfig.pull_down_en = GPIO_PULLDOWN_DISABLE;
    ioConfig.intr_type = GPIO_INTR_DISABLE;
    gpioInitResult_ = gpio_config(&ioConfig) == ESP_OK ? PLATFORM_OK : PLATFORM_FAIL;
    if (gpioInitResult_ == PLATFORM_OK) {
        ESP_EARLY_LOGI(
            Tag,
            "Initialized on I2C%u, address=0x%02X, READY GPIO=%d",
            static_cast<unsigned>(BusId),
            static_cast<unsigned>(config_.i2c.deviceAddress),
            static_cast<int>(config_.readyPin)
        );
    } else {
        ESP_EARLY_LOGE(Tag, "Failed to initialize READY GPIO %d", static_cast<int>(config_.readyPin));
    }
}

template<size_t BusId>
PlatformErr ADS1115<BusId>::ReadRaw(uint8_t channel, int16_t& value) {
    if (channel >= ChannelCount) {
        ESP_LOGE(Tag, "Invalid channel: %u", static_cast<unsigned>(channel));
        return PLATFORM_INVALID_PARAM;
    }

    const uint16_t channelConfig = static_cast<uint16_t>(
        ConfigStartSingleConversion | (static_cast<uint16_t>(SingleEndedMuxBase + channel) << MuxShift)
        | ConfigGainTwoVolts | ConfigSingleShot | ConfigDataRate860Sps | ConfigDisableComparator
    );

    PlatformErr result = WriteRegister(ConfigRegister, channelConfig);
    if (result != PLATFORM_OK) {
        ESP_LOGE(
            Tag,
            "Failed to start conversion on channel %u: err=%d",
            static_cast<unsigned>(channel),
            static_cast<int>(result)
        );
        return result;
    }
    result = WaitForConversion();
    if (result != PLATFORM_OK) {
        ESP_LOGE(
            Tag,
            "Conversion failed on channel %u: err=%d",
            static_cast<unsigned>(channel),
            static_cast<int>(result)
        );
        return result;
    }

    uint16_t rawValue = 0U;
    result = ReadRegister(ConversionRegister, rawValue);
    if (result == PLATFORM_OK) {
        value = static_cast<int16_t>(rawValue);
        ESP_LOGD(Tag, "Channel %u raw=%d", static_cast<unsigned>(channel), static_cast<int>(value));
    } else {
        ESP_LOGE(Tag, "Failed to read channel %u: err=%d", static_cast<unsigned>(channel), static_cast<int>(result));
    }
    return result;
}

template<size_t BusId>
PlatformErr ADS1115<BusId>::ReadAll(ADS1115_Values& values) {
    if (config_.currentSenseResistanceOhm <= 0.0F || config_.currentSenseGain <= 0.0F) {
        lastResult_ = PLATFORM_INVALID_PARAM;
        ESP_LOGE(
            Tag,
            "Invalid current measurement config: resistance=%f, gain=%f",
            static_cast<double>(config_.currentSenseResistanceOhm),
            static_cast<double>(config_.currentSenseGain)
        );
        return lastResult_;
    }

    int16_t raw[ChannelCount] {};
    for (uint8_t channel = 0U; channel < ChannelCount; ++channel) {
        const PlatformErr result = ReadRaw(channel, raw[channel]);
        if (result != PLATFORM_OK) {
            lastResult_ = result;
            return result;
        }
    }

    const float channel0 = static_cast<float>(raw[0]) * VoltsPerLsb;
    const float channel1 = static_cast<float>(raw[1]) * VoltsPerLsb;
    const float channel2 = static_cast<float>(raw[2]) * VoltsPerLsb;
    const float channel3 = static_cast<float>(raw[3]) * VoltsPerLsb;

    values.outputVoltage = channel0 * config_.dividerRatio;
    values.outputCurrent = channel1 / (config_.currentSenseResistanceOhm * config_.currentSenseGain);
    values.input1Voltage = channel2 * config_.dividerRatio;
    values.input2Voltage = channel3 * config_.dividerRatio;

    values_ = values;
    lastResult_ = PLATFORM_OK;
    ESP_LOGD(
        Tag,
        "Values: in1=%.3fV, in2=%.3fV, out=%.3fV, current=%.3fA",
        static_cast<double>(values.input1Voltage),
        static_cast<double>(values.input2Voltage),
        static_cast<double>(values.outputVoltage),
        static_cast<double>(values.outputCurrent)
    );
    return PLATFORM_OK;
}

template<size_t BusId>
[[nodiscard]] const ADS1115_Values& ADS1115<BusId>::Values() const {
    return values_;
}

template<size_t BusId>
[[nodiscard]] PlatformErr ADS1115<BusId>::GpioInitResult() const {
    return gpioInitResult_;
}

template<size_t BusId>
[[nodiscard]] PlatformErr ADS1115<BusId>::LastResult() const {
    return lastResult_;
}

template<size_t BusId>
void ADS1115<BusId>::SetSampleCallback(typename ADS1115<BusId>::SampleCallback callback, void* context) {
    sampleCallback_ = callback;
    sampleContext_ = context;
}

template<size_t BusId>
void ADS1115<BusId>::SetFailureRetryCycles(uint32_t cycles) {
    failureRetryCycles_ = cycles;
}

template<size_t BusId>
void ADS1115<BusId>::Update() {
    samplePending_ = false;
    if (sampleCallback_ == nullptr)
        return;
    if (retryCyclesRemaining_ > 0U) {
        --retryCyclesRemaining_;
        return;
    }

    scheduledResult_ = ReadAll(scheduledValues_);
    samplePending_ = true;
    if (scheduledResult_ != PLATFORM_OK) {
        retryCyclesRemaining_ = failureRetryCycles_;
    }
}

template<size_t BusId>
void ADS1115<BusId>::Handle() {
    if (!samplePending_ || sampleCallback_ == nullptr)
        return;
    sampleCallback_(sampleContext_, scheduledResult_, scheduledValues_);
    samplePending_ = false;
}

template<size_t BusId>
PlatformErr ADS1115<BusId>::WriteRegister(uint16_t reg, uint16_t value) {
    const uint8_t data[2] {
        static_cast<uint8_t>(value >> 8U),
        static_cast<uint8_t>(value & 0xFFU),
    };
    return i2cAgent_.MemWrite(reg, 1U, I2C_MemoryAddressEndian::BigEndian, data, sizeof(data));
}

template<size_t BusId>
PlatformErr ADS1115<BusId>::ReadRegister(uint16_t reg, uint16_t& value) {
    uint8_t data[2] {};
    const PlatformErr result = i2cAgent_.MemRead(reg, 1U, I2C_MemoryAddressEndian::BigEndian, data, sizeof(data));
    if (result == PLATFORM_OK) {
        value = static_cast<uint16_t>((static_cast<uint16_t>(data[0]) << 8U) | data[1]);
    }
    return result;
}

template<size_t BusId>
PlatformErr ADS1115<BusId>::WaitForConversion() {
    if (config_.conversionPollIntervalUs == 0U) {
        ESP_LOGE(Tag, "Conversion poll interval must not be zero");
        return PLATFORM_INVALID_PARAM;
    }

    const uint32_t maximumPolls = (config_.conversionTimeoutUs / config_.conversionPollIntervalUs) + 1U;
    for (uint32_t poll = 0U; poll < maximumPolls; ++poll) {
        uint16_t configValue = 0U;
        const PlatformErr result = ReadRegister(ConfigRegister, configValue);
        if (result != PLATFORM_OK) {
            ESP_LOGE(Tag, "Failed to poll conversion status: err=%d", static_cast<int>(result));
            return result;
        }
        if ((configValue & ConfigConversionReady) != 0U) {
            return PLATFORM_OK;
        }
        esp_rom_delay_us(config_.conversionPollIntervalUs);
    }
    ESP_LOGE(Tag, "Conversion timeout after %u us", static_cast<unsigned>(config_.conversionTimeoutUs));
    return PLATFORM_TIMEOUT;
}

#endif // FINEMOTE_ADS1115_HPP
