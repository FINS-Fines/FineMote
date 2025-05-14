// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "Task.h"
#include "DSHOT.h"

DSHOT dshot;

uint16_t target{0};

void UserTask1() {
  static uint32_t initTick = HAL_GetTick();
  if(HAL_GetTick() - initTick > 6000) {
    target = 100;
  }
  dshot.SetTargrtThrottle(target);
}
TASK_EXPORT(UserTask1);

void HAL_DMA_XferCpltCallback(DMA_HandleTypeDef *hdma)
{
  if(hdma == &hdma_tim8_up){
    dshot.isTxFinished = true;
    dshot.Switch2Receive();
    dshot.Receive();
  }
  else if(hdma == &hdma_tim1_up) {
    dshot.Decode();
  }
}

void DshotInit() {
  hdma_tim8_up.XferCpltCallback = &HAL_DMA_XferCpltCallback;
  hdma_tim1_up.XferCpltCallback = &HAL_DMA_XferCpltCallback;
  dshot.StartTransmit();
  dshot.InitReceive();
}
