/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "Scheduler.h"
#include "Encoders/MPT_45H.hpp"
#include "Odrive.hpp"

/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
auto motorControllers = Amplifier<1>();

Odrive<2> motorA(DIRECT_POSITION, motorControllers, 0x01);
MPT_45H<2> encoderA(0x03);

void Setup() {
}


void MotorTask() {
    motorA.SetTargetAngle(720.0f);
}

/**
 * @brief 主循环，优先级低于定时器中断，不确定执行频率
 */
void Loop() {
    // Do something
    HAL_Delay(1000);
}

#ifdef __cplusplus
}
#endif

void MainRTLoop() {
    HAL_IWDG_Refresh(&hiwdg);
    DeviceBase::DevicesHandle();
    FineMoteScheduler();
    MotorTask();
}

/*****  不要修改以下代码 *****/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if (htim == &TIM_Control) {
        MainRTLoop();
    }
}

#ifdef __cplusplus
}
#endif
