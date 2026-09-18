/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BQ25756_TYPES_HPP
#define FINEMOTE_BQ25756_TYPES_HPP

#include <cstdint>

#include "Bus/I2C_Config.hpp"
#include "driver/gpio.h"

enum class BQ25756_FastChargeThreshold : uint8_t
{
    Percent30 = 0U,
    Percent55 = 1U,
    Percent66_7 = 2U,
    Percent71_4 = 3U,
};

enum class BQ25756_AutoRechargeThreshold : uint8_t
{
    Percent93 = 0U,
    Percent94_3 = 1U,
    Percent95_2 = 2U,
    Percent97_6 = 3U,
};

enum class BQ25756_AdcSampleSpeed : uint8_t
{
    Bits15 = 0U,
    Bits14 = 1U,
    Bits13 = 2U,
};

enum class BQ25756_ChargeState : uint8_t
{
    NotCharging = 0U,
    TrickleCharge = 1U,
    Precharge = 2U,
    FastCharge = 3U,
    TaperCharge = 4U,
    Reserved = 5U,
    TopOff = 6U,
    TerminationDone = 7U,
};

enum BQ25756_Fault : uint8_t
{
    BQ25756_FaultNone = 0x00U,
    BQ25756_FaultDriverSupply = 0x02U,
    BQ25756_FaultSafetyTimer = 0x04U,
    BQ25756_FaultThermalShutdown = 0x08U,
    BQ25756_FaultBatteryOvervoltage = 0x10U,
    BQ25756_FaultBatteryOvercurrent = 0x20U,
    BQ25756_FaultInputOvervoltage = 0x40U,
    BQ25756_FaultInputUndervoltage = 0x80U,
};

enum BQ25756_MeasurementField : uint16_t
{
    BQ25756_MeasurementBatteryVoltage = 1U << 0U,
    BQ25756_MeasurementBatteryCurrent = 1U << 1U,
    BQ25756_MeasurementInput = 1U << 2U,
    BQ25756_MeasurementBatteryTemperature = 1U << 3U,
    BQ25756_MeasurementFeedbackVoltage = 1U << 4U,
    BQ25756_MeasurementChargeState = 1U << 5U,
    BQ25756_MeasurementFaultStatus = 1U << 6U,
    BQ25756_MeasurementWatchdog = 1U << 7U,
};

struct BQ25756_DeviceConfig
{
    I2C_DeviceConfig i2c {
        I2C_DeviceAddressWidth::Bits7,
        0x6BU,
        100000U,
    };
    gpio_num_t readyPin = GPIO_NUM_5;
    float batterySenseResistanceOhm = 0.005F;
    float inputSenseResistanceOhm = 0.002F;
    float feedbackDividerRatio3S = 12.52F / 1.536F;
    float feedbackDividerRatio6S = 25.2F / 1.536F;
};

struct BQ25756_ChargerConfig
{
    uint16_t chargeVoltageMv = 25000U;
    uint16_t chargeCurrentMa = 10000U;
    uint16_t inputCurrentLimitMa = 12000U;
    uint16_t inputVoltageLimitMv = 21600U;
    uint16_t prechargeCurrentMa = 1600U;
    uint16_t terminationCurrentMa = 2000U;

    bool terminationEnabled = true;
    BQ25756_FastChargeThreshold fastChargeThreshold =
        BQ25756_FastChargeThreshold::Percent71_4;
    bool prechargeEnabled = true;

    uint8_t topOffTimer = 0U;
    uint8_t watchdogTimer = 0U;
    bool safetyTimerEnabled = true;
    uint8_t safetyTimer = 0U;
    bool slowSafetyTimerInDpm = false;
    uint8_t constantVoltageTimer = 2U;
    BQ25756_AutoRechargeThreshold autoRechargeThreshold =
        BQ25756_AutoRechargeThreshold::Percent97_6;
    bool chargeOnWatchdogExpiry = false;

    bool cePinEnabled = true;
    bool ichgPinEnabled = true;
    bool ilimHizPinEnabled = true;
    bool powerGoodPinEnabled = true;
    bool statusPinsEnabled = true;

    bool highImpedanceEnabled = false;
    bool batteryLoadEnabled = false;
    bool inputLoadEnabled = false;
    bool pfmEnabled = false;
    bool reverseModeEnabled = false;
    bool mpptEnabled = false;
    bool temperaturePinEnabled = true;

    bool adcEnabled = true;
    bool adcOneShot = false;
    BQ25756_AdcSampleSpeed adcSampleSpeed = BQ25756_AdcSampleSpeed::Bits13;
    bool adcRunningAverage = true;
    bool adcInitializeAverage = true;
    bool inputCurrentAdcEnabled = true;
    bool batteryCurrentAdcEnabled = true;
    bool inputVoltageAdcEnabled = true;
    bool batteryVoltageAdcEnabled = true;
    bool temperatureAdcEnabled = true;
    bool feedbackVoltageAdcEnabled = true;
};

struct BQ25756_Measurements
{
    uint16_t validFields = 0U;
    float batteryVoltageV = 0.0F;
    float batteryCurrentA = 0.0F;
    float inputVoltageV = 0.0F;
    float inputCurrentA = 0.0F;
    float batteryTemperatureC = 0.0F;
    float feedbackVoltageV = 0.0F;
    BQ25756_ChargeState chargeState = BQ25756_ChargeState::NotCharging;
    uint8_t faultFlags = BQ25756_FaultNone;
    bool watchdogExpired = false;
};

#endif // FINEMOTE_BQ25756_TYPES_HPP
