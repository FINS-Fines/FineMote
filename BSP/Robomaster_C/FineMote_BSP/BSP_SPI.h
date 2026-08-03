/******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BSP_SPI_H
#define FINEMOTE_BSP_SPI_H

#include "Board.h"

template<size_t ID>
static constexpr size_t SPI_QUEUE_SIZE = 32;

class BSP_SPIs {
public:
    static BSP_SPIs &GetInstance() {
        static BSP_SPIs instance;
        return instance;
    }

private:
    BSP_SPIs() {
        PeripheralsInit::GetInstance();
    }
};

template<size_t ID>
class BSP_SPI {
public:
    static BSP_SPI &GetInstance() {
        static BSP_SPI instance;
        return instance;
    }

    static void CS_Low(uint8_t deviceID) {
        const SPI_CS& cs = SPI_CSList[deviceID];
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
    }

    static void CS_High(uint8_t deviceID) {
        const SPI_CS& cs = SPI_CSList[deviceID];
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
    }

    void Transmit(uint8_t *pTxData, uint16_t Size);
    void Receive(uint8_t *pRxData, uint16_t Size);
    void TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

    static constexpr size_t GET_QUEUE_SIZE() {
        return SPI_QUEUE_SIZE<ID>;
    }

private:
    BSP_SPI() {
        static_assert(
            ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr,
            "Invalid SPI ID"
        );
        BSP_SPIs::GetInstance();
    }
};

template<size_t ID>
void BSP_SPI<ID>::Transmit(uint8_t *pTxData, uint16_t Size) {
    HAL_SPI_Transmit_DMA(BSP_SPIList[ID], pTxData, Size);
}

template<size_t ID>
void BSP_SPI<ID>::Receive(uint8_t *pRxData, uint16_t Size) {
    HAL_SPI_Receive_DMA(BSP_SPIList[ID], pRxData, Size);
}

template<size_t ID>
void BSP_SPI<ID>::TransmitReceive(
    uint8_t *pTxData,
    uint8_t *pRxData,
    uint16_t Size) {
    HAL_SPI_TransmitReceive_DMA(BSP_SPIList[ID], pTxData, pRxData, Size);
}

#endif //FINEMOTE_BSP_SPI_H
