/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_ROBOMASTER_A_H
#define FINEMOTE_ROBOMASTER_A_H

#include "main.h"
#include "adc.h"
#include "can.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "iwdg.h"
#include "stm32f4xx_it.h"

int main();

class PeripheralsInit {

    PeripheralsInit() {
        main();
    }

public:
    PeripheralsInit(const PeripheralsInit &) = delete;

    PeripheralsInit &operator=(const PeripheralsInit &) = delete;

    static PeripheralsInit &GetInstance() {
        static PeripheralsInit instance;
        return instance;
    }
};

/**
 * UART Definitions
 */
constexpr UART_HandleTypeDef *BSP_UARTList[] = {nullptr, &huart1, &huart2, &huart3, nullptr, nullptr, &huart6, &huart7, &huart8};
constexpr size_t UART_BUS_MAXIMUM_COUNT = sizeof(BSP_UARTList) / sizeof(BSP_UARTList[0]) - 1;

/**
 * RS485 Definitions
 */
constexpr size_t BSP_RS485UARTIndexList[] = {NULL, NULL, NULL, 3, NULL, NULL, 6, 7, 8};
constexpr size_t RS485_BUS_MAXIMUM_COUNT = sizeof(BSP_RS485UARTIndexList) / sizeof(BSP_RS485UARTIndexList[0]) - 1;

/**
 * CAN Definitions
 */
constexpr CAN_HandleTypeDef *BSP_CANList[] = {nullptr, &hcan1, &hcan2};
constexpr size_t CAN_BUS_MAXIMUM_COUNT = sizeof(BSP_CANList) / sizeof(BSP_CANList[0]) - 1;

/**
 * DHSOT Definitions
 */
extern DMA_HandleTypeDef hdma_tim1_up;
extern DMA_HandleTypeDef hdma_tim8_up;

#define DSHOT_TRANSMIT_DMA_HANDLE hdma_tim1_up
#define DSHOT_RECEIVE_DMA_HANDLE hdma_tim8_up
#define DSHOT_TRANSMIT_DMA_TIM htim1
#define DSHOT_RECEIVE_DMA_TIM htim8
#define DSHOT_PIN_NUMBER 16

inline GPIO_TypeDef *const BSP_DHOSTPortList[DSHOT_PIN_NUMBER + 1] = {
    nullptr, GPIOI, GPIOI, GPIOI, GPIOI, GPIOA, GPIOA, GPIOA, GPIOA, //对应Z-S
    GPIOI,   GPIOH, GPIOH, GPIOH, GPIOD, GPIOD, GPIOD, GPIOD};       //对应A-H

constexpr size_t BSP_DHOSTPinList[DSHOT_PIN_NUMBER + 1] = {
    0,           GPIO_PIN_2,  GPIO_PIN_7,  GPIO_PIN_6,  GPIO_PIN_5,
    GPIO_PIN_3,  GPIO_PIN_2,  GPIO_PIN_1,  GPIO_PIN_0,  GPIO_PIN_0,
    GPIO_PIN_12, GPIO_PIN_11, GPIO_PIN_10, GPIO_PIN_15, GPIO_PIN_14,
    GPIO_PIN_13, GPIO_PIN_12};

/**
 * PWM Definitions
 */
constexpr struct {
  uint32_t TIM_CHANNEL;
  TIM_HandleTypeDef *TIM_Handle = nullptr;
  uint16_t TIM_Frequency = 168; // Default frequency
} BSP_PWMList[18] = {
  {0, nullptr, 0},
  {TIM_CHANNEL_1, &htim2, 84},//S
  {TIM_CHANNEL_2, &htim2, 84},//T
  {TIM_CHANNEL_3, &htim2, 84},//U
  {TIM_CHANNEL_4, &htim2, 84},//V
  {TIM_CHANNEL_1, &htim4, 84},//H
  {TIM_CHANNEL_2, &htim4, 84},//G
  {TIM_CHANNEL_3, &htim4, 84},//E
  {TIM_CHANNEL_4, &htim4, 84},//F
  {TIM_CHANNEL_1, &htim5, 84},//D
  {TIM_CHANNEL_2, &htim5, 84},//C
  {TIM_CHANNEL_3, &htim5, 84},//B
  {TIM_CHANNEL_4, &htim5, 84},//A
  /*** TIM8被DSHOT回传DMA征用 不再使用W-Z的PWM ***/
  {0, nullptr, 0},//W
  {0, nullptr, 0},//X
  {0, nullptr, 0},//Y
  {0, nullptr, 0},//Z
  {TIM_CHANNEL_1, &htim12, 84}  // BUZZER_PWM
};

/**
 * SPI Definitions
 */

/**
 * BUZZER Definitions
 */
#define BUTTON_PRESSED_STATE GPIO_PIN_SET
#define BUZZER_PWM_ID 17
#define BUZZER_PERIPHERAL

/**
 * LED Definitions
 */
inline GPIO_TypeDef *const BSP_LED_PortList[9] = {nullptr, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG, GPIOG};
constexpr size_t BSP_LED_PinList[9] = {0, LED_G1_Pin,  LED_G2_Pin,  LED_G3_Pin,  LED_G4_Pin, LED_G5_Pin,  LED_G6_Pin,  LED_G7_Pin,  LED_G8_Pin};
#define LED_FLOW
#define LED_GPIO_Port   LED_RED_GPIO_Port
#define LED_Pin         LED_RED_Pin
#define LED_PERIPHERAL

#define TIM_Control htim7

typedef struct {
    SPI_HandleTypeDef *spiHandle;
    DMA_HandleTypeDef *rxDMAHandle;
    DMA_HandleTypeDef *txDMAHandle;
    TIM_HandleTypeDef *timHandleForHeat;
    uint32_t timChannelForHeat;
} SPI_WITH_DMA_t;

extern SPI_WITH_DMA_t spiWithDMA;

#endif
