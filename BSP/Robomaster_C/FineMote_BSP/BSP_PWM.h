/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BSP_PWM_H
#define FINEMOTE_BSP_PWM_H

#include "Board.h"

class BSP_PWMs {
public:
 static BSP_PWMs& GetInstance() {
  static BSP_PWMs instance;
  return instance;
 }

private:
 BSP_PWMs() {
  PeripheralsInit::GetInstance();
  BSP_PWMs_Setup();
 }
 void BSP_PWMs_Setup() {

 }
};


template <uint8_t ID>
class BSP_PWM {
public:
 static BSP_PWM& GetInstance() {
  static BSP_PWM instance;
  return instance;
 }

 void SetDutyCycle(float _dutyCycle) {
  uint32_t counterPeriod = __HAL_TIM_GET_AUTORELOAD(BSP_PWMList[ID].TIM_Handle);
  __HAL_TIM_SET_COMPARE(BSP_PWMList[ID].TIM_Handle, BSP_PWMList[ID].TIM_CHANNEL, _dutyCycle * counterPeriod);
 }

 void SetFrequency(uint32_t frequency) {
  uint32_t timerClock = BSP_PWMList[ID].TIM_Frequency * 1000000; // Convert MHz to Hz
  uint32_t prescaler = BSP_PWMList[ID].TIM_Handle->Instance->PSC;
  __HAL_TIM_SET_AUTORELOAD(BSP_PWMList[ID].TIM_Handle, (timerClock / (prescaler + 1)) / frequency);
 }


private:
 BSP_PWM() {
  static_assert(ID > 0 && ID < sizeof(BSP_PWMList) / sizeof(BSP_PWMList[0]) && BSP_PWMList[ID].TIM_Handle != nullptr, "Invalid PWM ID");
  BSP_PWMs::GetInstance();
  BSP_PWM_Setup();
 }

 void BSP_PWM_Setup() {
  //default frequency 50Hz, default compare 0
  SetDutyCycle(0);
  HAL_TIM_PWM_Start(BSP_PWMList[ID].TIM_Handle, BSP_PWMList[ID].TIM_CHANNEL);
 }
};


#endif
