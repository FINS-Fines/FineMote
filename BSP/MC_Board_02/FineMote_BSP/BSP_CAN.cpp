/*******************************************************************************
* Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "BSP_CAN.h"

#include "Bus/CAN_Base.hpp"

#ifdef __cplusplus
extern "C" {
#endif

/*void HAL_CAN_RxFifo0MsgPendingCallback(FDCAN_HandleTypeDef *hcan) {
    FineMoteAux_CAN<>::OnRxComplete(hcan);
}*/
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo0ITs){
    FineMoteAux_CAN<>::OnRxComplete(hfdcan);
}
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef *hfdcan, uint32_t RxFifo1ITs){
    FineMoteAux_CAN<>::OnRxComplete(hfdcan);
}

/*void HAL_CAN_TxMailbox0CompleteCallback(FDCAN_HandleTypeDef *hcan) {
    FineMoteAux_CAN<>::OnTxComplete(hcan);
}

void HAL_CAN_TxMailbox1CompleteCallback(FDCAN_HandleTypeDef *hcan) {
    FineMoteAux_CAN<>::OnTxComplete(hcan);
}

void HAL_CAN_TxMailbox2CompleteCallback(FDCAN_HandleTypeDef *hcan) {
    FineMoteAux_CAN<>::OnTxComplete(hcan);
}*/
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef *hfdcan){
    FineMoteAux_CAN<>::OnTxComplete(hfdcan);
}

#ifdef __cplusplus
}
#endif
