/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef BSP_CAN_H
#define BSP_CAN_H

#include "Board.h"
#include "Bus/CAN_Header.hpp"

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
        HAL_FDCAN_ActivateNotification(BSP_CANList[ID], FDCAN_IT_RX_FIFO0_NEW_MESSAGE,0);
        HAL_FDCAN_ActivateNotification(BSP_CANList[ID], FDCAN_IT_TX_COMPLETE | FDCAN_IT_TX_FIFO_EMPTY,0);

        FDCAN_FilterTypeDef fdcanFilter;
        // TODO: 检查fdcanFilter的配置情况
        /*canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
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
        canFilter.SlaveStartFilterBank = 14;*/
        fdcanFilter.IdType = FDCAN_STANDARD_ID;
        fdcanFilter.FilterType = FDCAN_FILTER_MASK;
        fdcanFilter.FilterIndex = 0;
        fdcanFilter.FilterConfig = FDCAN_FILTER_TO_RXFIFO0;
        fdcanFilter.FilterID1 = 0x0000;
        fdcanFilter.FilterID2 = 0x0000;
        HAL_FDCAN_ConfigFilter(BSP_CANList[ID], &fdcanFilter);

        HAL_FDCAN_Start(BSP_CANList[ID]);
    }

    void BSP_CAN_Setup() {
        PeriphralInit();
    }
};

template<uint8_t ID>
void BSP_CAN<ID>::Receive(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data) {
    FDCAN_RxHeaderTypeDef FDCAN_Header = {0};
    HAL_FDCAN_GetRxMessage(BSP_CANList[ID], FDCAN_RX_FIFO0, &FDCAN_Header, data);
    //FDCAN_Header转Header
    Header->ID = FDCAN_Header.Identifier;
    if(FDCAN_Header.IdType == FDCAN_STANDARD_ID) Header->IDE = CAN_ID_STD;
    else if(FDCAN_Header.IdType == FDCAN_EXTENDED_ID) Header->IDE = CAN_ID_EXT;
    if(FDCAN_Header.RxFrameType == FDCAN_DATA_FRAME) Header->RTR = CAN_RTR_DATA;
    else if(FDCAN_Header.RxFrameType == FDCAN_REMOTE_FRAME) Header->RTR = CAN_RTR_REMOTE;
    Header->DLC = (FDCAN_Header.DataLength);
}

template<uint8_t ID>
void BSP_CAN<ID>::Transmit(FineMote_CAN_HeaderTypeDef *Header, uint8_t *data) {
    //uint32_t TxMailbox = 0;
    FDCAN_TxHeaderTypeDef FDCAN_Header;
    //Header转FDCAN_Header
    FDCAN_Header.Identifier = Header->ID;
    if(Header->IDE == CAN_ID_STD) FDCAN_Header.IdType = FDCAN_STANDARD_ID;
    else if(Header->IDE == CAN_ID_EXT) FDCAN_Header.IdType = FDCAN_EXTENDED_ID;
    if(Header->RTR == CAN_RTR_DATA) FDCAN_Header.TxFrameType = FDCAN_DATA_FRAME;
    else if(Header->RTR == CAN_RTR_REMOTE) FDCAN_Header.TxFrameType = FDCAN_REMOTE_FRAME;
    FDCAN_Header.DataLength = (Header->DLC);//转成FDCAN_data_length_code
    FDCAN_Header.ErrorStateIndicator = FDCAN_ESI_ACTIVE;
    FDCAN_Header.BitRateSwitch = FDCAN_BRS_OFF;
    FDCAN_Header.FDFormat = FDCAN_CLASSIC_CAN;
    FDCAN_Header.TxEventFifoControl = FDCAN_STORE_TX_EVENTS;
    FDCAN_Header.MessageMarker = 0x00;

    HAL_FDCAN_AddMessageToTxFifoQ(BSP_CANList[ID], &FDCAN_Header, data);
}

#endif
