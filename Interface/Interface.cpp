/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"
#include "Tasks.hpp"

/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

    void Setup() {
        std::function<void(uint8_t *, uint16_t)> remoteDecodeFunc = [](uint8_t* data, uint16_t length){
            remote.Decode(data, length);
        };
        UARTBaseLite<3>::GetInstance().Bind(remoteDecodeFunc);

        std::function<void(uint8_t *, uint16_t)> fineSerialDecodeFunc = [](uint8_t* data, uint16_t length){
            fineSerial.Decode(data, length);
        };
        UARTBaseLite<5>::GetInstance().Bind(fineSerialDecodeFunc);
    }

    /**
     * @brief 主循环，优先级低于定时器中断，不确定执行频率
     */

    void Loop() {

    }

#ifdef __cplusplus
}
#endif

/*****  不要修改以下代码 *****/

#ifdef __cplusplus
extern "C" {
#endif

    void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
        if(htim == &TIM_Control) {
            HAL_IWDG_Refresh(&hiwdg);
            DeviceBase::DevicesHandle();

            RunAllTask();
        }
    }

#ifdef __cplusplus
}
#endif
