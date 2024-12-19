/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "UART_Base.h"
#include "RS485_Base.h"
#include "UARTBaseLite.h"

#ifdef UART_BASE_MODULE


std::map<UART_HandleTypeDef*,UART_Base*>& GetUartHandle_BusMap() {
    static std::map<UART_HandleTypeDef*,UART_Base*> uartHandle_BusMap;
    return uartHandle_BusMap;
}

#ifdef __cplusplus
extern "C" {
#endif

// 发送完成中断回调函数
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart) {
    if(huart == &huart5) {
        UARTBaseLite<5>::GetInstance().TxLoader();
        GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::WRITE);
    }
    else if(huart == &huart4){
        UARTBaseLite<4>::GetInstance().TxLoader();
        GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::WRITE);
    }
    else if(huart == &huart2){
        RS485_Base<2>::GetInstance().RxHandle();
    }
    else if(huart == &huart1){
        RS485_Base<1>::GetInstance().RxHandle();
    }
}

// 接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::READ);
}

// 出错中断回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::ERROR_CALL);

    if(HAL_UART_GetError(huart) & HAL_UART_ERROR_ORE)
    {
        __HAL_UART_FLUSH_DRREGISTER(huart);  //读DR寄存器，就可以清除ORE错误标志位
    }


    if(__HAL_UART_GET_FLAG(huart,UART_FLAG_ORE) != RESET)
    {
        __HAL_UART_CLEAR_OREFLAG(huart);
    }

    if (huart == &huart5) {
        UARTBaseLite<5>::GetInstance().RxHandle(0);
    }
    else if (huart == &huart4) {
        UARTBaseLite<5>::GetInstance().RxHandle(0);
    }
}

#ifdef __cplusplus
}
#endif

#endif //UART_BASE_MODULE