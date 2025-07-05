/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_PROJECTCONFIG_H
#define FINEMOTE_PROJECTCONFIG_H

#include <type_traits>

/******************************************************************************************************
1.根据cmakelist中选择的构建目标，导入对应BSP包
******************************************************************************************************/

#include "Board.h"

/******************************************************************************************************
2.处理各模块对BSP包的依赖关系，若满足全部依赖则启用模块，此处需要以注释形式给出应由BSP包实现的依赖条件的具体内容
 以@def 标记需要的宏定义，@variable 标记需要定义的变量，实现依赖条件后可由BSP包定义依赖条件对应宏定义
 各模块的部分配置选项也可以宏定义的方式提供于此处
 ******************************************************************************************************/

/**
 *  PeripheralsInit类为系统重要依赖，不满足可能导致外设初始化晚于设备的构造函数，从而导致对外设操作失效
 *  要求提供了由于确保HAL库初始化的处理对象，并且通过goto语句跳过main函数中由cube生成的初始化函数
 *
 */

// template <typename T, typename = void>
// struct is_complete : std::false_type {};
//
// template <typename T>
// struct is_complete<T, std::void_t<decltype(sizeof(T))>> : std::true_type {};
//
// template <>
// struct is_complete<void, void> : std::true_type {};
//
// template <typename T>
// struct is_complete<T, std::enable_if_t<std::is_function_v<T>>> : std::true_type {};
//
// template <typename T>
// inline constexpr bool is_complete_v = is_complete<T>::value;
//
//
//
// static_assert(is_complete_v<PeripheralsInit>, "PeripheralsInit must be completed in BSP.");

/**
 * BUZZER_PERIPHERAL PWM驱动的蜂鸣器
 * @def TIM_Buzzer 蜂鸣器对应PWM定时器
 * @def TIM_Buzzer_Channel  蜂鸣器对应定时器通道
 */
#if defined(BUZZER_PERIPHERAL)
#define BEEPMUSIC_MODULE
#endif

/**
 * LED_PERIPHERAL 单个LED灯
 * @def LED_GPIO_Port   LED对应端口组
 * @def LED_Pin         LED对应引脚号
 */
#if defined(LED_PERIPHERAL)
#define LED_MODULE
#endif

/******************************************************************************************************
 * 3. 功能选配
 *******************************************************************************************************/

// #define WITH_POV_EXAMPLE

#endif
