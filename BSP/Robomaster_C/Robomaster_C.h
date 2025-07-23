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

int main();

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

/**
 * DHSOT Definitions
 */

/**
 * PWM Definitions
 */
using PWMList_t =  struct {
  uint32_t TIM_CHANNEL;
  TIM_HandleTypeDef *TIM_Handle = nullptr;
  uint16_t TIM_Frequency = 168; // Default frequency
};

constexpr PWMList_t BSP_PWMList[9] = {
  {0, nullptr, 0},
  {TIM_CHANNEL_1, &htim1, 168},
  {TIM_CHANNEL_2, &htim1, 168},
  {TIM_CHANNEL_3, &htim1, 168},
  {TIM_CHANNEL_4, &htim1, 168},
  {TIM_CHANNEL_1, &htim8, 168},
  {TIM_CHANNEL_2, &htim8, 168},
  {TIM_CHANNEL_3, &htim8, 168},
  {TIM_CHANNEL_3, &htim4, 84}  // BUZZER_PWM
};

/**
 * BUZZER Definitions
 */
constexpr size_t BUZZER_PWM_ID = 8;

#define LED_GPIO_Port   LED_B_GPIO_Port
#define LED_Pin         LED_B_Pin
#define LED_PERIPHERAL

#define TIM_Control htim7
#define SPI_BMI088 hspi1 /** todo */

#define USER_I2C hi2c2

typedef struct {
    SPI_HandleTypeDef* spiHandle;
    DMA_HandleTypeDef* rxDMAHandle;
    DMA_HandleTypeDef* txDMAHandle;
    TIM_HandleTypeDef* timHandleForHeat;
    uint32_t timChannelForHeat;
}SPI_WITH_DMA_t;
extern SPI_WITH_DMA_t spiWithDMA;

#endif
