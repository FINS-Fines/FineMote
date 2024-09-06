/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

/*****  示例1 *****/
#include "LED.h"

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
#include "BeepMusic.h"

void Task2() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}

/*****  示例3 *****/
#include "Chassis.h"
#include "Chassis_differential.h"
#include "RMD_L_40xx_v3.h"
#include "Motor4010.h"
#include "Motor4315.h"
#include "dji_group_agent.h"
#include "M3508_temp.h"

/*
constexpr PID_Param_t speedPID = {0.25f, 0.002f, 0.3f, 2000, 2000};

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
*/

//constexpr PID_Param_t anglePID = {2.0f, 0.0005f, 0.05f, 2000, 2000};
constexpr PID_Param_t speedPID = {0.15f, 0.002f, 0.3f, 2000, 16384};

auto wheelControllers = CreateControllers<PID, 2>(speedPID);


#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
#define TORQUE_2_POSITION {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Position}
#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}

#define M3508_left_para {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed, 19.f}
#define M3508_right_para {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed, -19.f}

M3508<1> LMotor(M3508_left_para, wheelControllers[0], 0x201);
M3508<2> RMotor(M3508_right_para, wheelControllers[0], 0x202);

//首先调取底盘类的构建器，然后使用提供的电机添加函数，将上文构建的电机指针传入构建器，最后由构建器返回构建好的底盘类对象
Chassis_d chassis = Chassis_d::Build().
        AddLMotor(LMotor).
        AddRMotor(RMotor).
        Build();

/*auto swerveControllers = CreateControllers<Amplifier<1>, 4>();
#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
RMD_L_40xx_v3<1> m1(DIRECT_POSITION, swerveControllers[0], 0x241);*/

#include "RadioMaster_Zorro.h"

RadioMaster_Zorro remote;

/**
 * @brief 底盘根据遥控器数据运动
 */
void Task3() {
    constexpr float speedLimit = 2;
    chassis.ChassisSetVelocity(remote.GetInfo().rightCol * speedLimit, remote.GetInfo().leftRol * PI);
}

/*****  示例4 *****/

void Task4() {
}

/**
 * @brief 用户初始化
 */
#include "FZMotion.h"

FZMotion motion;

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    std::function<void(uint8_t *, uint16_t)> remoteDecodeFunc = [](uint8_t* data, uint16_t length){
        remote.Decode(data, length);
    };
    UARTBaseLite<3>::GetInstance().Bind(remoteDecodeFunc);

    std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
        motion.Decode(data, length);
    };
//    UARTBaseLite<5>::GetInstance().Bind(decodeFunc);
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
