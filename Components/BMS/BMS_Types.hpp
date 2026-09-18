/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BMS_TYPES_HPP
#define FINEMOTE_BMS_TYPES_HPP

#include <cstddef>
#include <cstdint>

#include "Bus/platform_error.h"
#include "Power/BQ25756/BQ25756_Types.hpp"

enum class BMS_BatteryProfile : uint8_t
{
    None,
    Cells3,
    Cells6,
};

enum class BMS_ChargeState : uint8_t
{
    Unknown,
    Charging,
    Completed,
    NotCharging,
    Fault,
};

enum class BMS_EstimateMethod : uint8_t
{
    Uncalibrated,
    SessionCharge,
    FuelGaugeRecommended,
};

enum BMS_SampleSource : uint8_t
{
    BMS_SourcePcf8574 = 1U << 0U,
    BMS_SourceAds1115 = 1U << 1U,
    BMS_SourceBq25756 = 1U << 2U,
};

struct BMS_AdsMeasurements
{
    float input1Voltage = 0.0F;
    float input2Voltage = 0.0F;
    float outputVoltage = 0.0F;
    float outputCurrent = 0.0F;
};

struct BMS_ChargerMeasurements
{
    uint16_t validFields = 0U;
    float batteryVoltage = 0.0F;
    float batteryCurrent = 0.0F;
    float inputVoltage = 0.0F;
    float inputCurrent = 0.0F;
    float batteryTemperature = 0.0F;
    float feedbackVoltage = 0.0F;
    uint8_t chargeState = 0U;
    uint8_t faultFlags = 0U;
    bool watchdogExpired = false;
};

struct BMS_Sample
{
    uint32_t timestampMs = 0U;
    uint8_t updatedMask = 0U;
    uint8_t validMask = 0U;
    uint8_t pcf8574 = 0U;
    BMS_AdsMeasurements ads {};
    BMS_ChargerMeasurements charger {};
};

struct BMS_PowerStatistics
{
    bool sampleValid = false;
    bool chargeStateValid = false;
    bool charging = false;
    float currentPowerW = 0.0F;
    double chargingEnergyWh = 0.0;
    double nonChargingEnergyWh = 0.0;
    double unknownStateEnergyWh = 0.0;
};

struct BMS_BatteryEstimate
{
    bool sampleValid = false;
    bool batteryCurrentValid = false;
    bool chargeStateValid = false;
    bool charging = false;
    float batteryVoltage = 0.0F;
    float batteryCurrent = 0.0F;
    bool voltageSocValid = false;
    float voltageSocPercent = 0.0F;
    double sessionChargedAh = 0.0;
    double bootNetAh = 0.0;
    double bootChargedAh = 0.0;
    double bootDischargedAh = 0.0;
    double integratedSeconds = 0.0;
    BMS_EstimateMethod method = BMS_EstimateMethod::Uncalibrated;
};

struct BMS_ChargeCurvePoint
{
    uint32_t elapsedSeconds = 0U;
    float batteryVoltage = 0.0F;
    float batteryCurrent = 0.0F;
};

struct BMS_PowerCurvePoint
{
    uint32_t elapsedSeconds = 0U;
    float voltage = 0.0F;
    float current = 0.0F;
    float power = 0.0F;
};

struct BMS_Status
{
    bool started = false;
    BMS_BatteryProfile detectedProfile = BMS_BatteryProfile::None;
    BMS_BatteryProfile configuredProfile = BMS_BatteryProfile::None;
    BMS_ChargeState chargeState = BMS_ChargeState::Unknown;
    uint8_t validSources = 0U;
    uint32_t droppedSamples = 0U;
    PlatformErr lastDriverResult = PLATFORM_NOT_INITIALIZED;
};

struct BMS_Config
{
    int chargeStatusGpio = 48;
    uint32_t chargeRetryIntervalMs = 10000U;
    uint32_t watchdogResetIntervalMs = 10000U;
    uint32_t curveSampleIntervalMs = 10000U;
    uint32_t statusLogIntervalMs = 5000U;
};

struct BMS_DriverCallbacks
{
    void* context = nullptr;
    PlatformErr (*initializeChargeStatusOutput)(void* context, int gpio) = nullptr;
    PlatformErr (*setChargeStatusOutput)(void* context, int gpio, bool charging) = nullptr;
    PlatformErr (*configureCharger)(void* context,
                                    const BQ25756_ChargerConfig& config) = nullptr;
    PlatformErr (*startCharging)(void* context) = nullptr;
    PlatformErr (*stopCharging)(void* context) = nullptr;
    PlatformErr (*resetWatchdog)(void* context) = nullptr;
};

#endif // FINEMOTE_BMS_TYPES_HPP
