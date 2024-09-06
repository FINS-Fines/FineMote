/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "UARTBaseLite.h"

#include "UART_Base.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if (huart == &huart6) {
        UARTBaseLite<6>::GetInstance().RxHandle(Size);
    }
    else if (huart == &huart3) {
        UARTBaseLite<3>::GetInstance().RxHandle(Size);
    }
    else if (huart == &huart1){
        UARTBaseLite<1>::GetInstance().RxHandle(Size);
    }
}
