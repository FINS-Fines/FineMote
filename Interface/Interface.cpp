/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

/*****  示例1 *****/
#include "Emm28.h"
#include "LED.h"
#include "BeepMusic.h"
#include "Chassis.h"
#include "RMD_L_40xx_v3.h"
#include "Motor4010.h"
#include "Motor4315.h"
#include "HO3507.h"
#include "Odrive.h"
#include "Manipulator.h"
#include "D28_5_485.h"
#include "RadioMaster_Zorro.h"
#include "FineSerial.h"

/**
 * @brief LED闪烁
 */
void Task1() {
    static int cnt = 0;
    cnt++;
    if(cnt > 1000) {
        cnt = 0;
        LED::Toggle();
    }
}

/*****  示例2 *****/
/**
 * @brief 音乐播放与切换
 */

void Task2() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}

/*****  示例3 *****/

/*****  底盘部分  *****/
constexpr PID_Param_t speedPID = {0.3f, 0.005f, 0.33f, 2000, 2000};

auto wheelControllers = CreateControllers<PID, 4>(speedPID);
auto swerveControllers = CreateControllers<Amplifier<1>, 4>();

//构建组成底盘的各个电机
#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);
Motor4010<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x143);
Motor4010<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x142);
Motor4010<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x141);

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
Motor4315<1> SBRMotor(DIRECT_POSITION, swerveControllers[0], 0x04);
Motor4315<1> SBLMotor(DIRECT_POSITION, swerveControllers[1], 0x03);
Motor4315<1> SFLMotor(DIRECT_POSITION, swerveControllers[2], 0x02);
Motor4315<1> SFRMotor(DIRECT_POSITION, swerveControllers[3], 0x01);


// constexpr PID_Param_t speedPID = {0.1f, 0.003f, 0.1f, 2000, 2000};
//
// auto wheelControllers = CreateControllers<PID, 4>(speedPID);
// auto swerveControllers = CreateControllers<Amplifier<1>, 4>();
//
// #define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
// #define TORQUE_2_POSITION {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Position}
// #define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
//
// RMD_L_40xx_v3<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x242);
// RMD_L_40xx_v3<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x244);
// RMD_L_40xx_v3<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x246);
// RMD_L_40xx_v3<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x248);
//
// RMD_L_40xx_v3<1> SFRMotor(DIRECT_POSITION, swerveControllers[0], 0x241);
// RMD_L_40xx_v3<1> SFLMotor(DIRECT_POSITION, swerveControllers[1], 0x243);
// RMD_L_40xx_v3<1> SBLMotor(DIRECT_POSITION, swerveControllers[2], 0x245);
// RMD_L_40xx_v3<1> SBRMotor(DIRECT_POSITION, swerveControllers[3], 0x247);

// 首先调取底盘类的构建器，然后使用提供的电机添加函数，将上文构建的电机指针传入构建器，最后由构建器返回构建好的底盘类对象
Chassis chassis = Chassis::Build().
                  AddCFLMotor(CFLMotor).
                  AddCFRMotor(CFRMotor).
                  AddCBLMotor(CBLMotor).
                  AddCBRMotor(CBRMotor).
                  AddSFLMotor(SFLMotor).
                  AddSFRMotor(SFRMotor).
                  AddSBLMotor(SBLMotor).
                  AddSBRMotor(SBRMotor).
                  Build();


RoutePlanning route_planning(0.5);//Kp为误差补偿系数
bool isPathPointSet = false;
bool isMissionStart = false;


void Task3() {
    // if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == GPIO_PIN_RESET){
    //     return;
    // }

    isMissionStart = true;

    /*****  单次任务部分  *****/
    if (!FineSerial<5>::GetInstance().isCurrentTaskFinished){
        switch (FineSerial<5>::GetInstance().singleCommand){
        case SingleCommandType::NONE:
            break;
        case SingleCommandType::MISSION_START:
            // manipulator.GetInitCommand = true;
            FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
            break;
        case SingleCommandType::SET_PATH_POINT:
            route_planning.AddTarget(FineSerial<5>::GetInstance().path_point.x, 0,FineSerial<5>::GetInstance().path_point.y, 0,FineSerial<5>::GetInstance().path_point.yaw, 0, 5);
            FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
            break;
        case SingleCommandType::ODOMETRY_OFFSET:
            chassis.OffsetOdometry(FineSerial<5>::GetInstance().offset_data.x,FineSerial<5>::GetInstance().offset_data.y,FineSerial<5>::GetInstance().offset_data.yaw);
            route_planning.AddTarget(chassis.x - FineSerial<5>::GetInstance().offset_data.x, 0,
                                     chassis.y - FineSerial<5>::GetInstance().offset_data.y, 0,
                                     chassis.yaw - FineSerial<5>::GetInstance().offset_data.yaw, 0,4);
            FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
            break;
        case SingleCommandType::CHASSIS_STOP:
            chassis.ChassisSetVelocity(0, 0, 0);
            FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
            break;
        case SingleCommandType::END_EFFECTOR:
            break;
        }
    }

    /*****  循环任务部分  *****/
    chassis.ChassisSetVelocity(route_planning.FBVel, route_planning.LRVel, route_planning.RTVel);
    route_planning.Update(chassis.chassisPos[0][0], chassis.WCSVelocity[0][0], chassis.chassisPos[1][0],
                  chassis.WCSVelocity[1][0], chassis.chassisPos[2][0], chassis.WCSVelocity[2][0]);
}



uint8_t manipulatorCommand[28]{};
int cnt{0};

// void Task4(){
//     manipulatorCommand[0] = 0xAA;
//     memcpy(manipulatorCommand + 1, FineSerial<5>::GetInstance().manipulator_angle, 24);
//     manipulatorCommand[25] = static_cast<uint8_t>(FineSerial<5>::GetInstance().endEffectorState);
//     manipulatorCommand[26] = static_cast<uint8_t>(isMissionStart);
//     manipulatorCommand[27] = CRC8Calc(manipulatorCommand + 1, 26);
//     manipulatorCommand[28] = 0xBB;
//
//     cnt++;
//     if (cnt >= 5){
//         UARTBaseLite<4>::GetInstance().Transmit(manipulatorCommand, 29);
//         cnt = 0;
//     }
// }

void Task4(){
    cnt++;
    if (cnt >= 4){
        manipulatorCommand[0] = 0xAA;
        memcpy(manipulatorCommand + 1, FineSerial<5>::GetInstance().manipulator_angle, 24);
        manipulatorCommand[25] = static_cast<uint8_t>(FineSerial<5>::GetInstance().endEffectorState);
        manipulatorCommand[26] = CRC8Calc(manipulatorCommand + 1, 25);
        manipulatorCommand[27] = 0xBB;
        UARTBaseLite<4>::GetInstance().Transmit(manipulatorCommand, 28);
        cnt = 0;
    }
}

/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    // std::function<void(uint8_t *, uint16_t)> maniDecodeFunc = [](uint8_t* data, uint16_t length){
    //     remote.Decode(data, length);
    // };
    // UARTBaseLite<4>::GetInstance().Bind(maniDecodeFunc);

    // std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
    //     FineSerial<5>::GetInstance().Decode(data, length);
    // };
    // UARTBaseLite<5>::GetInstance().Bind(decodeFunc);

    RS485_Base<1>::GetInstance().SetDivisionFactor(4);
    RS485_Base<2>::GetInstance().SetDivisionFactor(100);
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
        // HAL_IWDG_Refresh(&hiwdg);
        DeviceBase::DevicesHandle();
        Task1();
        Task2();
        Task3();
        Task4();

        CAN_Bus<1>::TxLoader();
        CAN_Bus<2>::TxLoader();
    }
}

#ifdef __cplusplus
}
#endif
