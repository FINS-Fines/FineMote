/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

#include "LED.h"
#include "BeepMusic.h"
#include "Motor4010.h"
#include "Motor4315.h"
#include "RMD_L_40xx_v3.h"
#include "POV_Chassis.h++"
#include "RadioMaster_Zorro.h"
#include "FineSerial.h++"
#include "DS18B20_485.h"

/*****  示例1 *****/
/**
 * @brief LED闪烁
 */
void Task1() {
    static uint16_t cnt = 0;
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


/*****  电堆 温度管理子系统  *****/
/// {Temp[8], P_Stack} -> {P_cooler}
float temp_max = 0;
float temp_mean = 0;
float temp_min = 0;
#define NTC_NUM (8)
DS18B20_485<2> thermometer(0x01);
void Task3() {
    // 将数据从温度计从取出并写至全局
    auto temp_arr = thermometer.GetTemperature();
    // 计算最大、最小、平均温度
    for (int i = 0; i < NTC_NUM; i++) {
        temp_max = temp_max > temp_arr[i] ? temp_max : temp_arr[i];
        temp_min = temp_min < temp_arr[i] ? temp_min : temp_arr[i];
        temp_mean += temp_arr[i];
    }
    temp_mean /= NTC_NUM;
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
/// {P_demand, SOC} -> {P_stack}
float P_demand = 0;
float P_stack = 0;
float SOC = 0;
#define SOC_MAX 80;
#define SOC_MIN 20;
void Task6() {

}


/*****  电堆 系统安全监控管理子系统  *****/
/// 电堆巡检，氢气泄露，堆芯温度，进气压力
FineSerial fineSerial;
void Task7() {

}


/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    std::function<void(uint8_t *, uint16_t)> fineSerialDecodeFunc = [](uint8_t* data, uint16_t length){
        fineSerial.Decode(data, length);
    };
    UARTBaseLite<5>::GetInstance().Bind(fineSerialDecodeFunc);

    RS485_Base<1>::GetInstance().SetDivisionFactor(4);
    RS485_Base<2>::GetInstance().SetDivisionFactor(20);
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
