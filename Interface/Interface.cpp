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
#include "Motor4010.hpp"
/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
auto motorControllers = createAmplifiers<6>();

// 3个C4250电机, CAN通讯
Odrive<2> motorA(DIRECT_POSITION, motorControllers[0], 0x01);
Odrive<2> motorB(DIRECT_POSITION, motorControllers[1], 0x02);
Odrive<2> motorC(DIRECT_POSITION, motorControllers[2], 0x03);

// 3个RMD4010电机, CAN通讯
Motor4010<2> MotorD(DIRECT_POSITION, motorControllers[3],0x04);
Motor4010<2> MotorE(DIRECT_POSITION, motorControllers[4],0x05);
Motor4010<2> MotorF(DIRECT_POSITION, motorControllers[5],0x06);

// 6个MPT-45H编码器, RS485通讯
MPT_45H<2> encoderA(0x01);
MPT_45H<2> encoderB(0x02);
MPT_45H<2> encoderC(0x03);
MPT_45H<2> encoderD(0x04);
MPT_45H<2> encoderE(0x05);
MPT_45H<2> encoderF(0x06);

void Setup() {
}


void MotorOdriveTask() {
    // 3个电机的初始角度
    static float angleA = 0.0f;
    static float angleB = 0.0f;
    static float angleC = 0.0f;

    if(angleA < 240.0f){
        motorA.SetTargetAngle(angleA);
        angleA += 0.2;
    }
    if(angleB < 540.0f){
        motorB.SetTargetAngle(angleB);
        angleB += 0.2;
    }
    if(angleC < 540.0f){
        motorC.SetTargetAngle(angleC);
        angleC += 0.2;
    }
}

// static float angleD = 0.0f;

void Motor4010Task() {
    MotorE.SetTargetAngle(180);
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
    MotorOdriveTask();
    Motor4010Task();
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
