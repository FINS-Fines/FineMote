/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "Board.h"
#include "CAN_Base.hpp"
#include "CAN_Header.hpp"
class BSP_CANs {
public:
    static BSP_CANs &GetInstance() {
        static BSP_CANs instance;
        return instance;
    }

private:
    BSP_CANs() {
        PeripheralsInit::GetInstance();
        BSP_CANs_Setup();
    }

    void BSP_CANs_Setup() {
    }
};

template<uint8_t ID>
class BSP_CAN {
public:
    static BSP_CAN &GetInstance() {
        static BSP_CAN instance;
        return instance;
    }

    void Transmit(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data);

    void Receive(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data);

private:
    BSP_CAN() {
        static_assert(ID > 0 && ID <= CAN_BUS_MAXIMUM_COUNT && BSP_CANList[ID] != nullptr, "Invalid CAN ID");
        BSP_CANs::GetInstance();
        BSP_CAN_Setup();
    }

    void PeriphralInit() {
        HAL_CAN_ActivateNotification(BSP_CANList[ID], CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(BSP_CANList[ID], CAN_IT_TX_MAILBOX_EMPTY);

        CAN_FilterTypeDef canFilter;
        canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
        canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
        canFilter.FilterIdHigh = 0x0000;
        canFilter.FilterIdLow = 0x0000;
        canFilter.FilterMaskIdHigh = 0x0000;
        canFilter.FilterMaskIdLow = 0x0000;
        canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
        canFilter.FilterActivation = ENABLE;
        switch (ID) {
            case 1: canFilter.FilterBank = 0;
                break;
            case 2: canFilter.FilterBank = 14;
                break;
        }
        canFilter.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(BSP_CANList[ID], &canFilter);

        HAL_CAN_Start(BSP_CANList[ID]);
    }

    void BSP_CAN_Setup() {
        PeriphralInit();
    }
};

template<uint8_t ID>
void BSP_CAN<ID>::Receive(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data) {
  CAN_RxHeaderTypeDef CAN_Header = {0};
  HAL_CAN_GetRxMessage(BSP_CANList[ID], CAN_RX_FIFO0, &CAN_Header, data);
  if(CAN_Header.IDE == CAN_ID_STD){
    Header->IDE = CAN_ID_STD;
    Header->ID = CAN_Header.StdId;
  }
  else if(CAN_Header.IDE == CAN_ID_EXT){
    Header->IDE = CAN_ID_EXT;
    Header->ID = CAN_Header.ExtId;
  }
  Header->RTR = CAN_Header.RTR;
  Header->DLC = CAN_Header.DLC;
}

template<uint8_t ID>
void BSP_CAN<ID>::Transmit(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data) {
  uint32_t TxMailbox = 0;
  CAN_TxHeaderTypeDef CAN_Header;
  if(Header->IDE == CAN_ID_STD){
    CAN_Header.IDE = CAN_ID_STD;
    CAN_Header.StdId = Header->ID;
  }
  else if(Header->IDE == CAN_ID_EXT){
    CAN_Header.IDE = CAN_ID_EXT;
    CAN_Header.ExtId = Header->ID;
  }
  CAN_Header.RTR = Header->RTR;
  CAN_Header.DLC = Header->DLC;
  HAL_CAN_AddTxMessage(BSP_CANList[ID], &CAN_Header, data, &TxMailbox);
}

#endif
