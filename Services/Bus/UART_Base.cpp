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
#ifdef __ROBOMASTER_C
    if (huart == &huart6) {
        UARTBaseLite<2>::GetInstance().TxLoader();
        GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::WRITE);
    }
#endif
#ifdef __MC_BOARD
    if(huart == &huart5) {
        UARTBaseLite<5>::GetInstance().TxLoader();
        GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::WRITE);
    }
    else if(huart == &huart1){
        RS485_Base<1>::GetInstance().RxHandle();
    }
    else if(huart == &huart2){
        RS485_Base<2>::GetInstance().RxHandle();
    }
#endif

    //UART_Bus<0>::GetInstance().CallbackHandle(UART_Bus<0>::Callback_e::WRITE);
}
// 接收中断回调函数
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
    GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::READ);
    //UART_Bus<0>::GetInstance().CallbackHandle(UART_Bus<0>::Callback_e::READ);
}

// 出错中断回调函数
void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart) {
    GetUartHandle_BusMap()[huart]->CallbackHandle(UART_Base::Callback_e::ERROR_CALL);
#ifdef __ROBOMASTER_C
    if (huart == &huart6) {
        UARTBaseLite<2>::GetInstance().RxHandle(1);
    }
#endif
#ifdef __MC_BOARD
    if(huart->ErrorCode & HAL_UART_ERROR_ORE){
        // 重新启动接收
        if(huart == &huart3)
        {
            // 读取SR寄存器（自动获取状态）
            uint32_t status = huart->Instance->SR;

            // 读取DR寄存器（清除标志）
            uint32_t data = huart->Instance->SR;

            UARTBaseLite<3>::GetInstance().RxHandle(0);
        }
        else if(huart == &huart5)
        {
            // 读取SR寄存器（自动获取状态）
            uint32_t status = UART5->SR;

            // 读取DR寄存器（清除标志）
            uint32_t data = UART5->DR;
            UARTBaseLite<5>::GetInstance().RxHandle(0);
        }
    }
#endif
}

#ifdef __cplusplus
}
#endif

#endif //UART_BASE_MODULE