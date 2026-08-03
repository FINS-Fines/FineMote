/******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#include "BSP_SPI.h"
#include "Bus/SPI_Base.hpp"

#ifdef __cplusplus
extern "C" {
    #endif

    void HAL_SPI_TxRxCpltCallback(SPI_HandleTypeDef *hspi) {
        FineMoteAux_SPI<>::OnTxRxComplete(hspi);
    }

    void HAL_SPI_TxCpltCallback(SPI_HandleTypeDef *hspi) {
        FineMoteAux_SPI<>::OnTxRxComplete(hspi);
    }

    void HAL_SPI_RxCpltCallback(SPI_HandleTypeDef *hspi) {
        FineMoteAux_SPI<>::OnTxRxComplete(hspi);
    }

    #ifdef __cplusplus
}
#endif