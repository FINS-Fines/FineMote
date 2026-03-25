/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MC_BOARD_02_H
#define FINEMOTE_MC_BOARD_02_H

#include "main.h"
#include "adc.h"
#include "fdcan.h"
#include "dma.h"
// #include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "stm32h7xx_it.h"

int main();

class PeripheralsInit
{
    PeripheralsInit()
    {
        main();
    }

public:
    PeripheralsInit(const PeripheralsInit&) = delete;

    PeripheralsInit& operator =(const PeripheralsInit&
    )
    = delete;

    static PeripheralsInit
    & GetInstance()
    {
        static PeripheralsInit instance;
        return instance;
    }
};

/**
 * UART Definitions
 */
constexpr UART_HandleTypeDef* BSP_UARTList[] = {
    nullptr, &huart1, &huart2, &huart3, nullptr, &huart5, nullptr, &huart7, nullptr, nullptr, &huart10
};
constexpr size_t UART_BUS_MAXIMUM_COUNT = sizeof(BSP_UARTList) / sizeof(BSP_UARTList[0]) - 1;

/**
 * RS485 Definitions
 */
constexpr size_t BSP_RS485UARTIndexList[] = {0, 4, 2};
constexpr size_t RS485_BUS_MAXIMUM_COUNT = sizeof(BSP_RS485UARTIndexList) / sizeof(BSP_RS485UARTIndexList[0]) - 1;

inline GPIO_TypeDef* const BSP_RS485FlowCtrlPortList[3] = {nullptr, GPIOB, GPIOD};
constexpr uint16_t BSP_RS485FlowCtrlPinList[3] = {0, GPIO_PIN_14, GPIO_PIN_4};

/**
 * CAN Definitions
 */
constexpr FDCAN_HandleTypeDef* BSP_CANList[] = {nullptr, &hfdcan1, &hfdcan2, &hfdcan3};
constexpr size_t CAN_BUS_MAXIMUM_COUNT = sizeof(BSP_CANList) / sizeof(BSP_CANList[0]) - 1;

/**
 * DHSOT Definitions
 */


/**
 * PWM Definitions
 */
using PWMList_t = struct PWMList_t
{
    uint32_t TIM_CHANNEL;
    TIM_HandleTypeDef* TIM_Handle = nullptr;
    uint16_t TIM_Frequency = 168; // Default frequency
};

constexpr PWMList_t BSP_PWMList[6] = {
    {0, nullptr, 0},
    {TIM_CHANNEL_1, &htim2, 240},
    {TIM_CHANNEL_3, &htim2, 240},
    {TIM_CHANNEL_1, &htim1, 240},
    {TIM_CHANNEL_3, &htim1, 240},
    {TIM_CHANNEL_2, &htim12, 240} // BUZZER_PWM
};

/**
 * BUZZER Definitions
 */
constexpr size_t BUZZER_PWM_ID = 5;

#define LED_GPIO_Port   GPIOA
#define LED_Pin         GPIO_PIN_7
#define LED_PERIPHERAL

#define TIM_Heater htim3
#define TIM_Heater_Channel TIM_CHANNEL_2

#define TIM_Control htim7
#define hiwdg hiwdg1

typedef struct
{
    SPI_HandleTypeDef* spiHandle;
    DMA_HandleTypeDef* rxDMAHandle;
    DMA_HandleTypeDef* txDMAHandle;
    TIM_HandleTypeDef* timHandleForHeat;
    uint32_t timChannelForHeat;
} SPI_WITH_DMA_t;

extern SPI_WITH_DMA_t spiWithDMA;

/**
 * MICRO_ROS Definitions
 */
static constexpr bool WITH_MICRO_ROS = true;
static constexpr size_t MICRO_ROS_UART_ID = 1;

#endif
