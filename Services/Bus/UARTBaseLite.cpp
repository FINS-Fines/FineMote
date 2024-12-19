/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "UARTBaseLite.h"

#include "UART_Base.h"

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t Size) {
    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_FLUSH_DRREGISTER(huart);  //读DR寄存器，就可以清除ORE错误标志位
    }


    if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    if (huart == &huart5) {
        UARTBaseLite<5>::GetInstance().RxHandle(Size);
    }
    else if (huart == &huart3) {
        UARTBaseLite<3>::GetInstance().RxHandle(Size);
    }
    else if (huart == &huart2){
        UARTBaseLite<2>::GetInstance().RxHandle(Size);
    }
    else if (huart == &huart1){
        UARTBaseLite<1>::GetInstance().RxHandle(Size);
    }
}
