/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BSP_DSHOT_H
#define FINEMOTE_BSP_DSHOT_H

#include "Board.h"
#include "stm32f4xx_ll_gpio.h"

extern void DMA_XferCpltCallback(DMA_HandleTypeDef *hdma);

class BSP_DSHOTs {
public:
  static BSP_DSHOTs &GetInstance() {
    static BSP_DSHOTs instance;
    return instance;
  }

  void TransmitMessage(uint32_t* data, GPIO_TypeDef* GPIO_N, uint32_t DataLength) {
    HAL_DMA_Start_IT(&DSHOT_DMA_HANDLE, (uint32_t)data, (uint32_t)&GPIO_N->BSRR, DataLength);
  }

private:
  BSP_DSHOTs() {
    PeripheralsInit::GetInstance();
    BSP_DSHOTs_Setup();
  }

  void BSP_DSHOTs_Setup() {
    DSHOT_DMA_HANDLE.XferCpltCallback = &DMA_XferCpltCallback;
    __HAL_TIM_ENABLE_DMA(&DSHOT_DMA_TIM, TIM_DMA_UPDATE);
    HAL_TIM_Base_Start(&DSHOT_DMA_TIM);
  }
};


template <uint16_t ID>
class BSP_DSHOT {
public:
  static BSP_DSHOT& GetInstance() {
    static BSP_DSHOT instance;
    return instance;
  }

private:
  BSP_DSHOT() {
    static_assert(ID > 0 && ID < sizeof(BSP_DHOSTPinList) / sizeof(BSP_DHOSTPinList[0]) && BSP_DHOSTPinList[ID] != 0, "Invalid DSHOT ID");
    BSP_DSHOTs::GetInstance();
    BSP_DSHOT_Setup();
  }

  void BSP_DSHOT_Setup() {
    LL_GPIO_SetPinMode(BSP_DHOSTPortList[ID],BSP_DHOSTPinList[ID],LL_GPIO_MODE_OUTPUT);
  }
};


#endif
