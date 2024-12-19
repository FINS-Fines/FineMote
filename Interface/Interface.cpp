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
#include "Emm28.h"
#include "HO3507.h"
#include "Odrive.h"
// #include "FZMotion.h"
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

/*****  机械臂部分  *****/
struct ManipulatorAngle{
    float angleA{0};
    float angleB{0};
    float angleC{0};
    float angleD{0};
    float angleE{0};
    float angleF{0};
    uint8_t endEffecctor{0};
}__packed manipulator_angle;

auto manipulatorControllers = CreateControllers<Amplifier<1>, 7>();
#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}

Emm28<2>  DMotor(DIRECT_POSITION,manipulatorControllers[3],0x0400);
Emm28<2>  EMotor(DIRECT_POSITION,manipulatorControllers[4],0x0500);
HO3507<2> FMotor(DIRECT_POSITION,manipulatorControllers[5],0x07);
HO3507<2> GMotor(DIRECT_POSITION,manipulatorControllers[6],0x08);
Odrive<2> AMotor(DIRECT_POSITION,manipulatorControllers[0],0x02);
Odrive<2> BMotor(DIRECT_POSITION,manipulatorControllers[1],0x03);
Odrive<2> CMotor(DIRECT_POSITION,manipulatorControllers[2],0x04);

D28_485<2> CEncoder(0x03);
D28_485<2> DEncoder(0x04);
D28_485<2> EEncoder(0x01);

Manipulator manipulator(&AMotor,&BMotor,&CMotor,&DMotor,&EMotor,&FMotor,&GMotor,&CEncoder,&DEncoder,&EEncoder);

void Task3() {
    manipulator.GetInitCommand = true;
    if(manipulator.isInitFinished)
    {
        manipulator.SetAngle(manipulator_angle.angleA,manipulator_angle.angleB,manipulator_angle.angleC,
                             manipulator_angle.angleD,manipulator_angle.angleE,manipulator_angle.angleF);
        manipulator.SetEndEffectorAngle(static_cast<bool>(manipulator_angle.endEffecctor));
    }
}

void Task4(){

}

/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    std::function<void(uint8_t *, uint16_t)> DecodeFunc = [](uint8_t* data, uint16_t length){
        if(length != 28){return;}
        if(data[0] == 0xAA && data[27] == 0xBB)
        {
            if(CRC8Calc(data+1,25) == data[26]){
                memcpy(&manipulator_angle,data+1,25);
            }
        }
    };
    UARTBaseLite<4>::GetInstance().Bind(DecodeFunc);

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
