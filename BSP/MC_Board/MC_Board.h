/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MC_BOARD_H
#define FINEMOTE_MC_BOARD_H

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


#ifdef __cplusplus
extern "C" {
#endif

extern int main();

#ifdef __cplusplus
}
#endif

class HALInit {

    HALInit() {
        main();
    };

public:
    HALInit(const HALInit &) = delete;

    HALInit &operator=(const HALInit &) = delete;

    static HALInit &GetInstance() {
        static HALInit instance;
        return instance;
    }
};

#define HAL_INIT_HANDLE


#define UART_PERIPHERAL
constexpr UART_HandleTypeDef *BSP_UARTList[] = {nullptr, &huart1, &huart2, &huart3, nullptr, &huart5};
constexpr size_t UART_BUS_MAXIMUM_COUNT = sizeof(BSP_UARTList) / sizeof(BSP_UARTList[0]) - 1;

#define RS485_PERIPHERAL
constexpr size_t BSP_RS485UARTIndexList[] = {0, 1, 2};
constexpr size_t RS485_BUS_MAXIMUM_COUNT = sizeof(BSP_RS485UARTIndexList) / sizeof(BSP_RS485UARTIndexList[0]) - 1;

inline GPIO_TypeDef *const BSP_RS485FlowCtrlPortList[3] = {nullptr, GPIOC, GPIOB};
constexpr uint16_t BSP_RS485FlowCtrlPinList[3] = {0, GPIO_PIN_15, GPIO_PIN_3};

#define CAN_PERIPHERAL
constexpr CAN_HandleTypeDef *BSP_CANList[] = {nullptr, &hcan1, &hcan2};
constexpr size_t CAN_BUS_MAXIMUM_COUNT = sizeof(BSP_CANList) / sizeof(BSP_CANList[0]) - 1;

#define TIM_Buzzer htim2
#define TIM_Buzzer_Channel TIM_CHANNEL_4
#define BUZZER_PERIPHERAL

#define LED_GPIO_Port   GPIOC
#define LED_Pin         GPIO_PIN_0
#define LED_PERIPHERAL

#define TIM_Heater htim3
#define TIM_Heater_Channel TIM_CHANNEL_2

#define TIM_Control htim7

typedef struct {
    SPI_HandleTypeDef *spiHandle;
    DMA_HandleTypeDef *rxDMAHandle;
    DMA_HandleTypeDef *txDMAHandle;
    TIM_HandleTypeDef *timHandleForHeat;
    uint32_t timChannelForHeat;
} SPI_WITH_DMA_t;

extern SPI_WITH_DMA_t spiWithDMA;

#define IMU_PERIPHERAL

#endif //FINEMOTE_MC_BOARD_H
