/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_ROBOMASTER_C_H
#define FINEMOTE_ROBOMASTER_C_H

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "iwdg.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

#ifdef __cplusplus
extern "C" {
#endif

extern int main();

#ifdef __cplusplus
}
#endif

class PeripheralsInit{
/**
 *  @attention 此函数需要与cube生成的保持一致
 */
    PeripheralsInit() {
        main();
    }

public:
    PeripheralsInit(const PeripheralsInit &) = delete;
    PeripheralsInit& operator=(const PeripheralsInit&) = delete;
    static PeripheralsInit& GetInstance(){
        static PeripheralsInit instance;
        return instance;
    }
};

/**
 * UART Definitions
 */
constexpr UART_HandleTypeDef *BSP_UARTList[] = {nullptr, &huart6, &huart1, &huart3};
constexpr size_t UART_BUS_MAXIMUM_COUNT = sizeof(BSP_UARTList) / sizeof(BSP_UARTList[0]) - 1;

/**
 * CAN Definitions
 */
constexpr CAN_HandleTypeDef *BSP_CANList[] = {nullptr, &hcan1, &hcan2};
constexpr size_t CAN_BUS_MAXIMUM_COUNT = sizeof(BSP_CANList) / sizeof(BSP_CANList[0]) - 1;

#define TIM_Buzzer htim4
#define TIM_Buzzer_Channel TIM_CHANNEL_3
#define BUZZER_PERIPHERAL

#define LED_GPIO_Port   LED_B_GPIO_Port
#define LED_Pin         LED_B_Pin
#define LED_PERIPHERAL

#define TIM_Control htim7
#define SPI_BMI088 hspi1 /** todo */

#define USER_I2C hi2c2

typedef struct {
    TIM_HandleTypeDef* timerPtr; // 定时器HAL对象指针
    uint32_t channel;              // 定时器通道
    uint32_t activeFlag;
}PWM_UNIT_t;

extern PWM_UNIT_t pwmList[7];

typedef struct {
    SPI_HandleTypeDef* spiHandle;
    DMA_HandleTypeDef* rxDMAHandle;
    DMA_HandleTypeDef* txDMAHandle;
    TIM_HandleTypeDef* timHandleForHeat;
    uint32_t timChannelForHeat;
}SPI_WITH_DMA_t;
extern SPI_WITH_DMA_t spiWithDMA;

#endif
