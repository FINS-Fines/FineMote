/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef TASKS_HPP
#define TASKS_HPP

#include "ProjectConfig.h"

typedef void (*TaskFunc_t)();
#define TASK_EXPORT(n) __attribute__((used, section("Tasks"))) const TaskFunc_t __##n = n

#include "LED.h"
#include "BeepMusic.h"
#include "Motor4315.h"
#include "Motor4010.h"
#include "RMD_L_40xx_v3.h"
#include "Chassis/POV_Chassis.hpp"
#include "RadioMaster_Zorro.h"
#include "FineWarden/FineSerial.hpp"


/***** 任务1 *****/
/**
 * @brief LED闪烁˸
 */
void Task1() {
    static uint16_t cnt = 0;
    cnt++;
    if(cnt > 1000) {
        cnt = 0;
        LED::Toggle();
    }
}
TASK_EXPORT(Task1);

/***** 任务2 *****/
/**
 * @brief ���ֲ������л�
 */
void Task2() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}
TASK_EXPORT(Task2);

/***** 任务3 *****/
/**
 * @brief ���̸���ң���������˶�
 */

constexpr PID_Param_t speedPID = {0.23f, 0.008f, 0.3f, 2000, 2000};

auto wheelControllers = CreateControllers<PID, 4>(speedPID);
auto swerveControllers = CreateControllers<Amplifier<1>, 4>();

//������ɵ��̵ĸ������
#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);
Motor4010<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x143);
Motor4010<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x142);
Motor4010<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x141);

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position, true}
Motor4315<1> SBRMotor(DIRECT_POSITION, swerveControllers[0], 0x04);
Motor4315<1> SBLMotor(DIRECT_POSITION, swerveControllers[1], 0x03);
Motor4315<1> SFLMotor(DIRECT_POSITION, swerveControllers[2], 0x02);
Motor4315<1> SFRMotor(DIRECT_POSITION, swerveControllers[3], 0x01);

// constexpr PID_Param_t speedPID = {0.15f, 0.00145f, 0.0016f, 8000, 500};
// constexpr PID_Param_t positionInnerPID = {0.5f, 0.0f, 0.02f, 500, 500};
// constexpr PID_Param_t potisionOuterPID = {35.0f, 0.15f, 0.01f, 800, 800};
//
// auto wheelControllers = CreateControllers<PID, 4>(speedPID);
// auto swerveControllers = CreateControllers<CascadePID, 4>(potisionOuterPID, positionInnerPID);
//
// #define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
// #define TORQUE_2_POSITION {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Position, true}
//
// RMD_L_40xx_v3<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x242);
// RMD_L_40xx_v3<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x244);
// RMD_L_40xx_v3<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x246);
// RMD_L_40xx_v3<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x248);
//
// RMD_L_40xx_v3<1> SFRMotor(TORQUE_2_POSITION, swerveControllers[0], 0x241);
// RMD_L_40xx_v3<1> SFLMotor(TORQUE_2_POSITION, swerveControllers[1], 0x243);
// RMD_L_40xx_v3<1> SBLMotor(TORQUE_2_POSITION, swerveControllers[2], 0x245);
// RMD_L_40xx_v3<1> SBRMotor(TORQUE_2_POSITION, swerveControllers[3], 0x247);

constexpr float ROBOT_LENGTH = 0.2406f; //����0.240225f
constexpr float ROBOT_WIDTH = 0.24f; //�����
constexpr float WHEEL_DIAMETER = 0.0483; //4010ֱ��

auto chassis = POV_ChassisBuilder<Odom>( \
    WHEEL_DIAMETER, \
    Swerve_t{&SFRMotor, &CFRMotor,  ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, 180},
    Swerve_t{&SFLMotor, &CFLMotor,  ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2},
    Swerve_t{&SBLMotor, &CBLMotor, -ROBOT_LENGTH / 2,  ROBOT_WIDTH / 2},
    Swerve_t{&SBRMotor, &CBRMotor, -ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, 180}
    );

RadioMaster_Zorro remote;
FineSerial fineSerial;

void Task3() {
    constexpr float SPEED_LIMIT = 2.0f;

     if(remote.GetInfo().sC == RemoteControl::SWITCH_STATE_E::UP_POS) { //ң��ģʽ
         std::array<float, 3> targetV = {
             remote.GetInfo().rightCol * SPEED_LIMIT,
             -remote.GetInfo().rightRol * SPEED_LIMIT,
             -remote.GetInfo().leftRol * PI };
         chassis.SetVelocity(std::move(targetV));
     }
}
TASK_EXPORT(Task3);

void RunAllTasks() {
    extern const TaskFunc_t Tasks$$Base;
    extern const TaskFunc_t Tasks$$Limit;

    const TaskFunc_t *funcPtr;
    for(funcPtr = &Tasks$$Base; funcPtr < &Tasks$$Limit; funcPtr++) {
        (*funcPtr)();
    }
}

#endif //TASKS_HPP
