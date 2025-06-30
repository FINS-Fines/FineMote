/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#ifndef FINEMOTE_PWM_H
#define FINEMOTE_PWM_H

#include "ProjectConfig.h"

#ifdef PWM_MODULE

template<uint32_t timerID>
class TimerPWM {
private:
    TIM_HandleTypeDef* timerPtr = nullptr;

    TimerPWM() {
        PeripheralsInit::GetInstance();
        timerPtr = BSP_PWMTimList[timerID];
    }
    uint32_t timerClock;
public:
    static TimerPWM& GetInstance() {
        static TimerPWM instance;
        return instance;
    }

    // todo : 目前默认使用APB2总线，频率168M,之后要支持其他定时器
    void SetFrequency(uint32_t frequency) {
        timerClock = BSP_TIMFrequencyList[timerID];
        uint32_t prescaler = timerPtr->Instance->PSC;
        __HAL_TIM_SET_AUTORELOAD(timerPtr, (timerClock / (prescaler + 1)) / frequency);
    }

    void Start(uint32_t channel) {
        HAL_TIM_PWM_Start(timerPtr, channel);
    }

    void Stop(uint32_t channel) {
        HAL_TIM_PWM_Stop(timerPtr, channel);
    }

    TIM_HandleTypeDef* GetTimerHandle() const {
        return timerPtr;
    }
};

template<uint32_t timerID, uint32_t channelID>
class ChannelPWM {
private:
    TIM_HandleTypeDef* timerPtr = nullptr;
    uint32_t channel = 0;

    ChannelPWM() {
        constexpr uint32_t maxChannelCount = sizeof(BSP_PWMTimChannleList) / sizeof(BSP_PWMTimChannleList[0]);
        constexpr uint32_t maxTimerCount = sizeof(BSP_PWMTimList) / sizeof(BSP_PWMTimList[0]);

        static_assert(timerID > 0 && timerID <= maxTimerCount && BSP_PWMTimList[timerID] != nullptr, "Invalid timer ID");
        static_assert(channelID > 0 && channelID <= maxChannelCount, "Invalid channel ID");

        TimerPWM<timerID>::GetInstance();

        timerPtr = BSP_PWMTimList[timerID];
        channel = BSP_PWMTimChannleList[channelID];

        Start();
    }

public:
    static ChannelPWM& GetInstance() {
        static ChannelPWM instance;
        return instance;
    }

    void SetCompare(float _compare) {
        uint32_t counterPeriod = __HAL_TIM_GET_AUTORELOAD(timerPtr);
        __HAL_TIM_SET_COMPARE(timerPtr, channel, _compare * counterPeriod);
    }

    void SetFrequency(uint32_t frequency) {
        TimerPWM<timerID>::GetInstance().SetFrequency(frequency);
    }

    void Start() {
        HAL_TIM_PWM_Start(timerPtr, channel);
    }

    void Stop() {
        HAL_TIM_PWM_Stop(timerPtr, channel);
    }
};

#endif

#endif //FINEMOTE_PWM_H
