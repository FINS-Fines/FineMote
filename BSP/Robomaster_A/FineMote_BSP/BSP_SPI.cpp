// /*******************************************************************************
//  * Copyright (c) 2025.
//  * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
//  * All rights reserved.
//  ******************************************************************************/
//
// #include "Bus/SPI_Base.hpp"
// #include "BSP_SPI.h"
// #include "stm32f4xx_hal_gpio.h"
//
// #ifdef __cplusplus
// extern "C" {
// #endif
//
//  // 发送完成中断回调函数
//  void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hSPI) {
//   FineMoteAux_SPI<>::OnTxComplete(hSPI);
//  }
//
//  // 接收中断回调函数
//  void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hSPI) {
//   uint16_t receivedSize = hSPI->RxXferSize;
//   FineMoteAux_SPI<>::OnRxComplete(hSPI, receivedSize);
//  }
//
//  void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hSPI) {
//   FineMoteAux_SPI<>::OnTxComplete(hSPI);
//   uint16_t receivedSize = hSPI->RxXferSize;
//   FineMoteAux_SPI<>::OnRxComplete(hSPI, receivedSize);
//  }
//
//  // 出错中断回调函数
//  void HAL_SPI_ErrorCallback(SPI_HandleTypeDef *hSPI) {
//  }
//
//  void HAL_SPIEx_RxEventCallback(SPI_HandleTypeDef *hSPI, uint16_t size) {
//  }
//
// #ifdef __cplusplus
// }
// #endif
//
// template<uint8_t ID>
// void BSP_SPI<ID>::SelectDevice(uint16_t deviceID) {
//     GPIO_TypeDef* port = BSP_SPI_NSS_List[deviceID].GPIO_Port;
//     uint16_t pin = BSP_SPI_NSS_List[deviceID].GPIO_Pin;
//
//     // Set all NSS pins high
//     for (const auto& nss : BSP_SPI_NSS_List) {
//         HAL_GPIO_WritePin(nss.GPIO_Port, nss.GPIO_Pin, GPIO_PIN_SET);
//     }
//
//     // Set the selected device's NSS pin low
//     HAL_GPIO_WritePin(port, pin, GPIO_PIN_RESET);
// }
