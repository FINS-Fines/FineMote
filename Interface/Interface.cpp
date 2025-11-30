/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "Scheduler.h"
#include "Encoders/MPT_45H.hpp"
#include "Odrive.hpp"
#include "Manipulator.hpp"
/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

float initAngle = 0.0f;

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
auto motorControllers = createAmplifiers<3>();

Odrive<2> motorA(DIRECT_POSITION, motorControllers[0], 0x01);
Odrive<2> motorB(DIRECT_POSITION, motorControllers[1], 0x02);
Odrive<2> motorC(DIRECT_POSITION, motorControllers[2], 0x03);

// MPT_45H<2> encoderA(0x03);

void Setup() {
}


void MotorTask() {
    if(initAngle < 720.0f){
        motorA.SetTargetAngle(initAngle);
        motorB.SetTargetAngle(initAngle);
        motorC.SetTargetAngle(initAngle);
    }
    initAngle += 0.2;
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
