/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"
#include "BeepMusic.h"
#include "DeviceBase.h"
#include "LED.h"
#include "Motor4010.h"
#include "SEGGER_RTT.h"

#include "UARTBaseLite.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief 用户初始化
 */
#include "FZMotion.h"
extern FZMotion motion;

extern int __initial_sp;
extern int __stack_base;
extern int Stack_Size;
extern int __heap_base;
extern int __heap_limit;
void Setup() {
    std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
        motion.Decode(data, length);
    };
    UARTBaseLite<5>::GetInstance().Bind(decodeFunc);
}

/**
 * @brief 主循环，优先级低于定时器中断，不确定执行频率
 */
void TaskTest();//测试函数，会递归将栈占满然后再清空再占满再清空.....
float stack_usage_percent;
float TIM_Period_usage;
void Loop() {
    TaskTest();

}

#ifdef __cplusplus
}
#endif

/*****  示例1 *****/
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
#include "Chassis.h"
#include "RadioMaster_Zorro.h"

extern Chassis chassis;
extern RadioMaster_Zorro remote;

/**
 * @brief 底盘根据遥控器数据运动
 */
void Task3() {
    chassis.ChassisSetVelocity(remote.GetInfo().rightCol, remote.GetInfo().rightRol, remote.GetInfo().leftRol * 2 * PI * 2);
}
void TaskTest() {
    static int hahha = 0;
    HAL_Delay(10);
    hahha++;
    char buff[100];
    if(strlen(buff))
        buff;
    sprintf(buff, "");
//    SEGGER_RTT_WriteString(0, "tasktest cnt:");
    SEGGER_RTT_WriteString(0, buff);
    if (stack_usage_percent >= 40.0) {
        hahha = 0;
        return;
    }
    TaskTest();
}
/*****  示例4 *****/

/**
 * @brief 获取动捕数据
 */
void Task4() {

}
/*****  示例5 *****/

/**
 * @brief 获取栈利用率
 */
void Task5(){
    uint32_t sp = __get_MSP();  // 获取当前主栈指针（MSP）
    uint32_t stack_start = (uint32_t)&__stack_base; // 假设栈的起始地址
    uint32_t stack_size = (uint32_t)&Stack_Size;      // 假设栈的大小为8KB
    uint32_t stack_usage = stack_start + stack_size - sp;
    stack_usage_percent = (stack_usage / (float)stack_size) * 100;
}

/*****  不要修改以下代码 *****/

#ifdef __cplusplus
extern "C" {
#endif

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim == &TIM_Control) {
        uint32_t start_tick = SysTick->VAL;
        HAL_IWDG_Refresh(&hiwdg);
        DeviceBase::DevicesHandle();
        Task1();
        Task2();
        Task3();
        Task4();
        Task5();
        CAN_Bus<1>::TxLoader();
        CAN_Bus<2>::TxLoader();
        uint32_t end_tick =  SysTick->VAL;
        // 计算执行时间
        uint32_t execution_tick = start_tick - end_tick;
        float execution_time = (float)execution_tick/HAL_RCC_GetSysClockFreq();
        TIM_Period_usage = execution_time/0.001*100;

        //输出占用率
        char buff[100];
        sprintf(buff, "TIM_Period usage:%%%f,stack usage:%%%f\n" ,TIM_Period_usage,stack_usage_percent);
        SEGGER_RTT_WriteString(0, buff);

    }
}

#ifdef __cplusplus
};
#endif
