
/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_LED_H
#define FINEMOTE_LED_H

#include "ProjectConfig.h"

#ifdef LED_MODULE
class LED {
public:
    static void Toggle() {
        HAL_GPIO_TogglePin(LED_GPIO_Port, LED_Pin);
    }

    static void Flow(){
        static uint16_t currPos = 1;
        static uint16_t tick = 0;
        tick++;
        if(tick > 100) {
            tick = 0;
            HAL_GPIO_WritePin(BSP_LED_PortList[currPos], BSP_LED_PinList[currPos], GPIO_PIN_SET);
            currPos = currPos < 8 ? currPos + 1 : 1;
            HAL_GPIO_WritePin(BSP_LED_PortList[currPos], BSP_LED_PinList[currPos], GPIO_PIN_RESET);
        }
    }

    LED() {
        PeripheralsInit::GetInstance();
    }
};
#endif

#endif
