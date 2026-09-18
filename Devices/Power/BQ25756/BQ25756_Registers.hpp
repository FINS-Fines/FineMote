/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BQ25756_REGISTERS_HPP
#define FINEMOTE_BQ25756_REGISTERS_HPP

#include <cstdint>

namespace BQ25756_Registers
{
enum class Register : uint8_t
{
    ChargeVoltageLimit = 0x00U,
    ChargeCurrentLimit = 0x02U,
    InputCurrentDpmLimit = 0x06U,
    InputVoltageDpmLimit = 0x08U,
    PrechargeCurrentLimit = 0x10U,
    TerminationCurrentLimit = 0x12U,
    PrechargeTerminationControl = 0x14U,
    TimerControl = 0x15U,
    ThreeStageChargeControl = 0x16U,
    ChargerControl = 0x17U,
    PinControl = 0x18U,
    PowerPathReverseModeControl = 0x19U,
    MpptControl = 0x1AU,
    TsChargingRegionBehaviorControl = 0x1CU,
    ChargerStatus1 = 0x21U,
    FaultStatus = 0x24U,
    AdcControl = 0x2BU,
    AdcChannelControl = 0x2CU,
    InputCurrentAdc = 0x2DU,
    BatteryCurrentAdc = 0x2FU,
    InputVoltageAdc = 0x31U,
    BatteryVoltageAdc = 0x33U,
    TemperatureAdc = 0x37U,
    FeedbackVoltageAdc = 0x39U,
};

constexpr uint16_t ChargeVoltageMask = 0x001FU;
constexpr uint16_t ChargeCurrentMask = 0x07FCU;
constexpr uint16_t InputCurrentMask = 0x07FCU;
constexpr uint16_t InputVoltageMask = 0x3FFCU;
constexpr uint16_t PrechargeCurrentMask = 0x03FCU;
constexpr uint16_t TerminationCurrentMask = 0x03FCU;

constexpr uint8_t TerminationEnable = 0x08U;
constexpr uint8_t FastChargeThresholdMask = 0x06U;
constexpr uint8_t PrechargeEnable = 0x01U;

constexpr uint8_t TopOffTimerMask = 0xC0U;
constexpr uint8_t WatchdogTimerMask = 0x30U;
constexpr uint8_t SafetyTimerEnable = 0x08U;
constexpr uint8_t SafetyTimerMask = 0x06U;
constexpr uint8_t SafetyTimerSlowInDpm = 0x01U;
constexpr uint8_t ConstantVoltageTimerMask = 0x0FU;

constexpr uint8_t AutoRechargeThresholdMask = 0xC0U;
constexpr uint8_t WatchdogReset = 0x20U;
constexpr uint8_t CePinDisable = 0x10U;
constexpr uint8_t ChargeOnWatchdogExpiry = 0x08U;
constexpr uint8_t HighImpedanceEnable = 0x04U;
constexpr uint8_t BatteryLoadEnable = 0x02U;
constexpr uint8_t ChargeEnable = 0x01U;

constexpr uint8_t IchgPinEnable = 0x80U;
constexpr uint8_t IlimHizPinEnable = 0x40U;
constexpr uint8_t PowerGoodPinDisable = 0x20U;
constexpr uint8_t StatusPinsDisable = 0x10U;

constexpr uint8_t RegisterReset = 0x80U;
constexpr uint8_t InputLoadEnable = 0x40U;
constexpr uint8_t PfmEnable = 0x20U;
constexpr uint8_t ReverseModeEnable = 0x01U;
constexpr uint8_t MpptEnable = 0x01U;
constexpr uint8_t TemperaturePinEnable = 0x01U;

constexpr uint8_t AdcEnable = 0x80U;
constexpr uint8_t AdcOneShot = 0x40U;
constexpr uint8_t AdcSampleSpeedMask = 0x30U;
constexpr uint8_t AdcRunningAverage = 0x08U;
constexpr uint8_t AdcInitializeAverage = 0x04U;

constexpr uint8_t InputCurrentAdcDisable = 0x80U;
constexpr uint8_t BatteryCurrentAdcDisable = 0x40U;
constexpr uint8_t InputVoltageAdcDisable = 0x20U;
constexpr uint8_t BatteryVoltageAdcDisable = 0x10U;
constexpr uint8_t TemperatureAdcDisable = 0x04U;
constexpr uint8_t FeedbackVoltageAdcDisable = 0x02U;

constexpr uint8_t WatchdogExpired = 0x08U;
constexpr uint8_t ChargeStateMask = 0x07U;
constexpr uint16_t TemperatureAdcMask = 0x03FFU;
} // namespace BQ25756_Registers

#endif // FINEMOTE_BQ25756_REGISTERS_HPP
