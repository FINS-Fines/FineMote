/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "MC_Board.h"
#include "BeepMusic.h"

extern DMA_HandleTypeDef hdma_spi2_rx;
extern DMA_HandleTypeDef hdma_spi2_tx;

SPI_WITH_DMA_t spiWithDMA{&hspi2,&hdma_spi2_rx,&hdma_spi2_tx,
                          &htim3,TIM_CHANNEL_2};

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Setup() {
    HAL_TIM_Base_Start_IT(&TIM_Control);
    HAL_TIM_PWM_Start(&TIM_Buzzer,TIM_Buzzer_Channel);
    BeepMusic::MusicChannels[0].Play(3);
}

#ifdef __cplusplus
}
#endif
