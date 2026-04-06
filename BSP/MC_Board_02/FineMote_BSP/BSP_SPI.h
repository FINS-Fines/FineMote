#ifndef BSP_SPI_H
#define BSP_SPI_H

#include "Board.h"

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

    static void CS_Low(const SPI_CS& cs) {
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_RESET);
    }
    static void CS_High(const SPI_CS& cs) {
        HAL_GPIO_WritePin(cs.port, cs.pin, GPIO_PIN_SET);
    }

    void Transmit(uint8_t *pTxData, uint16_t Size);
    void Receive(uint8_t *pRxData, uint16_t Size);
    void TransmitReceive(uint8_t *pTxData, uint8_t *pRxData, uint16_t Size);

private:
    BSP_SPI() {
        static_assert(ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT&& BSP_SPIList[ID] != nullptr,"Invalid SPI ID");
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
void BSP_SPI<ID>::TransmitReceive(uint8_t *pTxData, uint8_t *pRxData,
                                  uint16_t Size) {
    HAL_SPI_TransmitReceive_DMA(BSP_SPIList[ID], pTxData, pRxData, Size);
}

// =============================================================================
// 模板特化区域：传输方式由 BSP 层决定，上层（SPI_Base）无需感知
// 默认：所有 SPI 异步接口走 DMA
// 特化：若某路 SPI 硬件未配 DMA 通道，回退为 IT，上层代码零改动
// =============================================================================

// ----- 示例：SPI2 无 DMA，异步回退为 IT -----
// template<>
// void BSP_SPI<2>::Transmit(uint8_t *pTxData, uint16_t Size) {
//     HAL_SPI_Transmit_IT(BSP_SPIList[2], pTxData, Size);
// }
//
// template<>
// void BSP_SPI<2>::Receive(uint8_t *pRxData, uint16_t Size) {
//     HAL_SPI_Receive_IT(BSP_SPIList[2], pRxData, Size);
// }
//
// template<>
// void BSP_SPI<2>::TransmitReceive(uint8_t *pTxData, uint8_t *pRxData,
//                                   uint16_t Size) {
//     HAL_SPI_TransmitReceive_IT(BSP_SPIList[2], pTxData, pRxData, Size);
// }

#endif // BSP_SPI_H