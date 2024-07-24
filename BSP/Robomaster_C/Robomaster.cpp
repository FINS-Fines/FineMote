/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Robomaster_C.h"

#include "BeepMusic.h"

PWM_UNIT_t pwmList[7] = {
{&htim1,TIM_CHANNEL_1},
{&htim1,TIM_CHANNEL_2},
{&htim1,TIM_CHANNEL_3},
{&htim1,TIM_CHANNEL_4},
{&htim8,TIM_CHANNEL_1},
{&htim8,TIM_CHANNEL_2},
{&htim8,TIM_CHANNEL_3}
};

UART_HandleTypeDef *uartHandleList[3] = {&huart3, &huart1, &huart6};
GPIO_TypeDef *rs485TxPortList[3] = {nullptr, nullptr, nullptr};
uint16_t rs485TxPinList[3] = {NULL, NULL, NULL};

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_rx;
extern DMA_HandleTypeDef hdma_spi1_tx;

SPI_WITH_DMA_t spiWithDMA{&hspi1,&hdma_spi1_rx,&hdma_spi1_tx,
                          &htim10,TIM_CHANNEL_1};

#ifdef __cplusplus
extern "C" {
#endif
extern int __initial_sp;
extern int __stack_base;
extern int Stack_Size;
extern int __heap_base;
extern int __heap_limit;
void BSP_Setup() {
    /* 堆栈溢出检测
   * 栈起始位置为0x200189c8，终止位置为0x200109c8
   * 保护区域为0x200109e8 ~ 0x200109c8
   */
    MPU_Region_InitTypeDef MPU_InitStruct;
    HAL_MPU_Disable();
    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.BaseAddress = (uint32_t)&__stack_base;
    MPU_InitStruct.Size = MPU_REGION_SIZE_16KB;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_NOT_SHAREABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL1;
    MPU_InitStruct.SubRegionDisable = 0x00;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_ENABLE;
    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
    HAL_NVIC_SetPriority(MemoryManagement_IRQn,0,0);
    HAL_TIM_Base_Start_IT(&TIM_Control);
    HAL_TIM_PWM_Start(&TIM_Buzzer,TIM_Buzzer_Channel);
    BeepMusic::MusicChannels[0].Play(3);
}

#ifdef __cplusplus
}
#endif
