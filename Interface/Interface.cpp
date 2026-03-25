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
//#include "RMD_L_40xx_v3.hpp"
/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

#define UART5_TX_DIVIDER 1000
#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
auto motorControllers = createAmplifiers<7>();

// C4250电机, CAN通讯
Odrive<2> motorA(DIRECT_POSITION, motorControllers[0], 0x01);
Odrive<2> motorB(DIRECT_POSITION, motorControllers[1], 0x02);
Odrive<2> motorC(DIRECT_POSITION, motorControllers[2], 0x03);

// RMD4010电机, CAN通讯
Motor4010<2> MotorD(DIRECT_POSITION, motorControllers[3],0x04);
Motor4010<2> MotorE(DIRECT_POSITION, motorControllers[4],0x05);
Motor4010<2> MotorF(DIRECT_POSITION, motorControllers[5],0x06);
Motor4010<2> EndEffector(DIRECT_POSITION, motorControllers[6],0x07);

// MPT-45H编码器, RS485通讯
MPT_45H<2> encoderA(0x01);
MPT_45H<2> encoderB(0x02);
MPT_45H<2> encoderC(0x03);
MPT_45H<2> encoderD(0x04);
MPT_45H<2> encoderE(0x05);
MPT_45H<2> encoderF(0x06);

// 测试用的帧数据
static uint8_t uart5_tx_frame[] = {
    0xAA, 0x0B, 0x18, 0xCD, 0xCC, 0xCC, 0x3D, 0xCD, 0xCC, 0x4C,
    0x3E, 0x9A, 0x99, 0x99, 0xBE, 0xCD, 0xCC, 0xCC, 0x3E, 0x00,
    0x00, 0x00, 0x3F, 0x9A, 0x99, 0x19, 0x3F, 0xEE, 0xBB
};


void Setup() {
    static ManipulatorUARTReceiver<5, MANIPULATOR_PAYLOAD_LENGTH> manipulator_uart_receiver;
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


void Motor4010Task() {
    MotorD.SetTargetAngle(720);
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
    static uint16_t uart5_tx_div_cnt = 0;
    HAL_IWDG_Refresh(&hiwdg);
    DeviceBase::DevicesHandle();
    FineMoteScheduler();
//    MotorOdriveTask();
    Motor4010Task();

    // Send UART5 frame at a reduced rate: once every UART5_TX_DIVIDER control loops.
    if(++uart5_tx_div_cnt >= UART5_TX_DIVIDER){
        uart5_tx_div_cnt = 0;
        UART_Base<5>::GetInstance().Transmit(uart5_tx_frame, sizeof(uart5_tx_frame));
    }
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
