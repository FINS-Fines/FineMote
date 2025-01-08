/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CAN_BASE_H
#define FINEMOTE_CAN_BASE_H

#include "ProjectConfig.h"

#ifdef CAN_BASE_MODULE

#include <map>
#include <queue>

/**
 * Todo:
 * 远程帧处理
 */

typedef struct {
    uint8_t DLC;
    uint8_t IDE;
    uint8_t RTR;
    uint32_t addr;
    uint8_t message[8];
} CAN_Package_t;

template <int busID>
class CAN_Bus {
public:

    static CAN_Bus &GetInstance() {
        static CAN_Bus instance;
        return instance;
    }

    CAN_Bus(const CAN_Bus &) = delete;
    CAN_Bus& operator=(const CAN_Bus&) = delete;

    static void RxHandle();
    static void TxReload();

    static bool LoadTxData(CAN_Package_t& txbuf) {
        if (dataQueue.size() < CAN_QUEUE_MAX_NUM) {
            dataQueue.push(txbuf);
            if (txOngoing == false) {
                TxReload();
                txOngoing = true;
            }
            return true;
        }
        return false;
    }

    static std::map<uint32_t, uint8_t *> &Getmap(){
        static std::map<uint32_t, uint8_t *> map;
        return map;
    }
    
private:
    CAN_Bus() {
        static_assert((busID > 0) && (busID <= CAN_BUS_MAXIMUM_COUNT), "Using Illegal CAN BUS");
        PeriphralInit();
    }

    void PeriphralInit() {
        HALInit::GetInstance();

        HAL_CAN_ActivateNotification(CAN_Buses[busID - 1], CAN_IT_RX_FIFO0_MSG_PENDING);
        HAL_CAN_ActivateNotification(CAN_Buses[busID - 1], CAN_IT_TX_MAILBOX_EMPTY);

        CAN_FilterTypeDef canFilter;
        canFilter.FilterMode = CAN_FILTERMODE_IDMASK;
        canFilter.FilterScale = CAN_FILTERSCALE_32BIT;
        canFilter.FilterIdHigh = 0x0000;
        canFilter.FilterIdLow = 0x0000;
        canFilter.FilterMaskIdHigh = 0x0000;
        canFilter.FilterMaskIdLow = 0x0000;
        canFilter.FilterFIFOAssignment = CAN_RX_FIFO0;
        canFilter.FilterActivation = ENABLE;
        switch (busID)
        {
            case 1:canFilter.FilterBank = 0; break;
            case 2:canFilter.FilterBank = 14; break;
        }
        canFilter.SlaveStartFilterBank = 14;
        HAL_CAN_ConfigFilter(CAN_Buses[busID - 1], &canFilter);

        HAL_CAN_Start(CAN_Buses[busID - 1]);
    }

    static std::queue<CAN_Package_t> dataQueue;
    static bool txOngoing;
};

template <int busID>
class CAN_Agent {
public:
    explicit CAN_Agent(uint32_t addr) : addr(addr) {
        static_assert((busID > 0) && (busID <= CAN_BUS_MAXIMUM_COUNT), "Using illegal CAN BUS");
        CAN_Bus<busID>::GetInstance().Getmap()[addr] = rxbuf;
    }

    void SetDLC(uint8_t _DLC) {
        txbuf.DLC = _DLC;
    }

/**
 * @brief CAN发送队列装填
 * @param _addr
 * @param config IDE | RTR
 * @param IDE CAN_ID_STD or CAN_ID_EXT
 * @param RTR CAN_RTR_DATA or CAN_RTR_REMOTE
 */
    void Send(uint32_t _addr, uint8_t config = CAN_ID_STD | CAN_RTR_DATA) {
        txbuf.addr = _addr;
        txbuf.IDE = config & CAN_ID_EXT;
        txbuf.RTR = config & CAN_RTR_REMOTE;

        CAN_Bus<busID>::LoadTxData(txbuf);
    }

    uint8_t& operator[](std::size_t index) {
        return txbuf.message[index];
    }

    uint8_t operator[] (std::size_t index) const {
        return rxbuf[index];
    }

    uint32_t addr;
    uint8_t rxbuf[8] = {0};

private:
    CAN_Package_t txbuf = {8};
};

class FineMoteAux_CAN {
public:
    template <int busID>
    static void Reloader(CAN_HandleTypeDef* hcan);

    template <int busID>
    static void Distributer(CAN_HandleTypeDef* hcan);

    static void Reload(CAN_HandleTypeDef* hcan);

    static void Distribute(CAN_HandleTypeDef* hcan);
};

#endif //CAN_BASE_MODULE
#endif //FINEMOTE_CAN_BASE_H
