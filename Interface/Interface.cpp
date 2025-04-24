/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

#include "BeepMusic.h"
#include "LED.h"
#include "PID.h"

#include "AirCooler.h"
#include "RS485Dev_PWMFan.hpp"
#include "RS485Dev_TempMonitor_8p.hpp"

/*****  示例1 *****/
/// @brief LED闪烁
void Task1() {
    static uint16_t cnt = 0;
    cnt++;
    if(cnt > 1000) {
        cnt = 0;
        LED::Toggle();
    }
}

/*****  示例2 *****/
/// @brief 音乐播放与切换
void Task2() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}

/*****  电堆 温度管理子系统  *****/
Fan<2> fan_00(0x02, 0x00, 500);
Fan<2> fan_01(0x02, 0x01, 500);
Fan<2> fan_02(0x02, 0x02, 500);
Fan<2> fan_03(0x02, 0x03, 500);
Fan<2> fan_04(0x03, 0x00, 500);
Fan<2> fan_05(0x03, 0x01, 500);
Fan<2> fan_06(0x03, 0x02, 500);
Fan<2> fan_07(0x03, 0x03, 500);
TempMonitor_8p<1> temp_monitor(0x04, 500);
constexpr PID_Param_t fan_pi_param = {-50.f, -0.05f, -0.80f, 2000, 100};
PID fan_ctrl_00{fan_pi_param};
PID fan_ctrl_01{fan_pi_param};
PID fan_ctrl_02{fan_pi_param};
PID fan_ctrl_03{fan_pi_param};
PID fan_ctrl_04{fan_pi_param};
PID fan_ctrl_05{fan_pi_param};
PID fan_ctrl_06{fan_pi_param};
PID fan_ctrl_07{fan_pi_param};
FuelCell::AirCooler cooler = FuelCell::AirCooler::Build().
    AddFan0(&fan_00, &fan_ctrl_00).
    AddFan1(&fan_01, &fan_ctrl_01).
    AddFan2(&fan_02, &fan_ctrl_02).
    AddFan3(&fan_03, &fan_ctrl_03).
    AddFan4(&fan_04, &fan_ctrl_04).
    AddFan5(&fan_05, &fan_ctrl_05).
    AddFan6(&fan_06, &fan_ctrl_06).
    AddFan7(&fan_07, &fan_ctrl_07).
    AddTempMonitor(&temp_monitor).Build();

void Task3() {

}


/*****  电堆 阴、阳极管理子系统  *****/
/// {P_stack} -> {O_supply, H_supply, H_pulse}
void Task4() {

}


/*****  电堆 负载管理子系统  *****/
/// {电位器} -> {P_demand}
void Task5() {

}


/*****  电堆 系统EMS监控管理子系统  *****/
void Task6() {

}


/*****  电堆 系统安全监控管理子系统  *****/
/// 电堆巡检，氢气泄露，堆芯温度，进气压力
// FineSerial fineSerial;
void Task7() {

}


/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    RS485_Base<1>::GetInstance().SetDivisionFactor(50);
    RS485_Base<2>::GetInstance().SetDivisionFactor(50);
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
        Task5();
        Task6();
        Task7();
    }
}

#ifdef __cplusplus
}
#endif
