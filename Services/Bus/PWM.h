/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#ifndef FINEMOTE_PWM_H
#define FINEMOTE_PWM_H

#include <cassert>
#include "ProjectConfig.h"

#ifdef PWM_MODULE

template<uint32_t deviceID>
class PWM {
private:
    TIM_HandleTypeDef* timerPtr;
    uint32_t channel;
    static inline uint32_t currentDeviceCount = 0;
    static inline uint32_t maxDeviceCount = sizeof(pwmList)/sizeof(pwmList[0]);

public:
    explicit PWM() {
        HALInit::GetInstance();
        assert(deviceID < maxDeviceCount);

        pwmList[deviceID].activeFlag = 0;

        currentDeviceCount++;
        timerPtr = pwmList[deviceID].timerPtr;
        channel = pwmList[deviceID].channel;
        pwmList[deviceID].activeFlag = 1;
        };
    ~PWM(){
            pwmList[deviceID].activeFlag = 0;
            currentDeviceCount--;
        };

    /**
     * @param _compare 0~1
     */
    void SetCompare(float _compare){
        if (_compare < 0) {
            _compare = 0;
        } else if (_compare > 1) {
            _compare = 1;
        }


        uint32_t counterPeriod = __HAL_TIM_GET_AUTORELOAD(timerPtr);
                __HAL_TIM_SetCompare (timerPtr,channel,_compare * counterPeriod);
    }

    float GetCompare() {
        uint32_t compareValue = __HAL_TIM_GET_COMPARE(timerPtr, channel);
        uint32_t arrValue = __HAL_TIM_GET_AUTORELOAD(timerPtr);

        if (arrValue == 0) {
            return 0.0f;  // 防止除零错误
        }

        return (float)compareValue / (float)arrValue;
    }

    // Attention : 此处只会修改同一个定时器的执行周期，这里对应的是 tim8，重复调用将被覆盖
    void SetFreq(uint32_t frequency) const {
        if (frequency == 0) return;

        volatile uint32_t timerClock;
        if (timerPtr->Instance == TIM1 || timerPtr->Instance == TIM8) {// 高级定时器
            timerClock = HAL_RCC_GetPCLK2Freq() * ((RCC->CFGR & RCC_CFGR_PPRE2) > 4 ? 2 : 1);
        } else {// 通用定时器
            timerClock = HAL_RCC_GetPCLK1Freq() * ((RCC->CFGR & RCC_CFGR_PPRE1) > 4 ? 2 : 1);
        }

        // 自动搜索一个合适的 prescaler 和 arr，使得 arr 不超过 65535
        volatile uint32_t prescaler = 1;
        volatile uint32_t arrValue = timerClock / (prescaler * frequency);

        while (arrValue > 0xFFFF && prescaler < 0xFFFF) {
            ++prescaler;
            arrValue = timerClock / (prescaler * frequency);
        }

        if (arrValue > 0xFFFF) arrValue = 0xFFFF;

        __HAL_TIM_SET_PRESCALER(timerPtr, prescaler - 1);
        __HAL_TIM_SET_AUTORELOAD(timerPtr, arrValue - 1);  // 注意减1，符合 HAL 初始化的方式
        __HAL_TIM_SET_COUNTER(timerPtr, 0);

        // 更新寄存器立即生效
        timerPtr->Instance->EGR |= TIM_EGR_UG;
    }

    void Start(){
        HAL_TIM_PWM_Start(timerPtr,channel);
    };
    void Stop(){
        HAL_TIM_PWM_Stop(timerPtr,channel);
    };

};
#endif

#endif //FINEMOTE_PWM_H
