/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BSP_SPI_H
#define FINEMOTE_BSP_SPI_H

#include "Board.h"

class BSP_SPIs {
public:
  static BSP_SPIs& GetInstance() {
    static BSP_SPIs instance;
    return instance;
  }

private:
  BSP_SPIs() {
    PeripheralsInit::GetInstance();
    BSP_SPIs_Setup();
  }
  void BSP_SPIs_Setup();
};


template <uint8_t ID>
class BSP_SPI {
public:
  static BSP_SPI& GetInstance() {
    static BSP_SPI instance;
    return instance;
  }

  void Transmit(uint8_t* data, uint16_t size);

  void Receive(uint8_t* data, uint16_t size);

  void TransmitReceive(uint8_t* txData, uint8_t* rxData, uint16_t size);

  void SelectDevice(uint16_t deviceID);

  void DeselectDevice(uint16_t deviceID);

private:
  BSP_SPI() {
    static_assert(ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr, "Invalid SPI ID");
    BSP_SPIs::GetInstance();
    BSP_SPI_Setup();
  }
  void BSP_SPI_Setup();
};

/**
 * BSP Specific Definitions
 */

inline void BSP_SPIs::BSP_SPIs_Setup() {

}

template<uint8_t ID>
void BSP_SPI<ID>::BSP_SPI_Setup() {

}

template<uint8_t ID>
void BSP_SPI<ID>::Transmit(uint8_t *data, uint16_t size) {
    HAL_SPI_Transmit_IT(BSP_SPIList[ID], data, size);
}

template<uint8_t ID>
void BSP_SPI<ID>::Receive(uint8_t *data, uint16_t size) {
    HAL_SPI_Receive_IT(BSP_SPIList[ID], data, size);
}

template<uint8_t ID>
void BSP_SPI<ID>::TransmitReceive(uint8_t* txData, uint8_t* rxData, uint16_t size) {
  HAL_SPI_TransmitReceive_IT(BSP_SPIList[ID], txData, rxData, size);
}

template<uint8_t ID>
void BSP_SPI<ID>::SelectDevice(uint16_t deviceID) {
      HAL_GPIO_WritePin(BSP_SPI_NSS_List[deviceID].GPIO_PORT, BSP_SPI_NSS_List[deviceID].GPIO_PIN, GPIO_PIN_RESET);
}

template<uint8_t ID>
void BSP_SPI<ID>::DeselectDevice(uint16_t deviceID) {
      HAL_GPIO_WritePin(BSP_SPI_NSS_List[deviceID].GPIO_PORT, BSP_SPI_NSS_List[deviceID].GPIO_PIN, GPIO_PIN_SET);
}

#endif
