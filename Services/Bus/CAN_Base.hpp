/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CAN_BASE_HPP
#define FINEMOTE_CAN_BASE_HPP

#include "etl/map.h"
#include "etl/queue.h"

#include <cstdint>

enum class CAN_ID_HeaderTypeDef : uint8_t {
    STD,
    EXT
};

enum class CAN_RTR_HeaderTypeDef : uint8_t {
    DATA,
    REMOTE
};

typedef struct{
    uint32_t ID;//CAN与FDCAN的ID没有区别
    uint8_t IDE;//使用CAN_identifier_type
    uint8_t RTR;//使用CAN_remote_transmission_request
    uint8_t DLC;//使用0~8的整数
} FineMote_CAN_HeaderTypeDef;//兼容CAN与FDCAN

#include "BSP_CAN.hpp"

template<uint8_t ID>
class BSP_CAN;

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

template<size_t ID>
class CAN_Base {
public:
    static CAN_Base &GetInstance() {
        static CAN_Base instance;
        return instance;
    }

    CAN_Base(const CAN_Base &) = delete;

    CAN_Base &operator=(const CAN_Base &) = delete;

    void RxHandle() {
        uint8_t tempBuf[8];
        FineMote_CAN_HeaderTypeDef Header;
        BSP_CAN<ID>::GetInstance().Receive(&Header, tempBuf);
        memcpy(rxBufferMap[Header.ID], tempBuf, Header.DLC);
    }

    void TxHandle() {
        if (!dataQueue.empty()) {
            FineMote_CAN_HeaderTypeDef Header;
            if (dataQueue.front().IDE == static_cast<uint32_t>(CAN_ID_HeaderTypeDef::STD) ||
                dataQueue.front().IDE == static_cast<uint32_t>(CAN_ID_HeaderTypeDef::EXT)) {
                Header.ID = dataQueue.front().addr;
            }
            Header.DLC = dataQueue.front().DLC;
            Header.IDE = dataQueue.front().IDE;
            Header.RTR = dataQueue.front().RTR;
            BSP_CAN<ID>::GetInstance().Transmit(&Header, dataQueue.front().message);
            dataQueue.pop();
        } else {
            isTxComplete = true;
        }
    }

    bool Transmit(CAN_Package_t &txbuf) {
        if (dataQueue.full()) {
            dataQueue.pop();
        }
        dataQueue.push(txbuf);
        if (isTxComplete == true) {
            TxHandle();
            isTxComplete = false;
        }
        return true;
    }

    void BindRxBuffer(const uint8_t *buffer, uint32_t addr) {
        rxBufferMap[addr] = const_cast<uint8_t *>(buffer);
    }

private:
    etl::map<uint32_t, uint8_t *, CAN_Parameters<ID>::CAN_MAP_SIZE> rxBufferMap;
    etl::queue<CAN_Package_t, CAN_Parameters<ID>::CAN_TX_QUEUE_SIZE> dataQueue;
    bool isTxComplete = true;

    CAN_Base() {
        BSP_CAN<ID>::GetInstance();
    }
};

template<size_t ID>
class CAN_Agent {
public:
    explicit CAN_Agent(uint32_t addr) : addr(addr) {
        static_assert(ID > 0 && ID <= CAN_BUS_MAXIMUM_COUNT && BSP_CANList[ID] != nullptr, "Using illegal CAN BUS");
        CAN_Base<ID>::GetInstance().BindRxBuffer(rxbuf, addr);
    }

    void SetDLC(uint8_t DLC) {
        txbuf.DLC = DLC;
    }

    /**
     * @brief CAN发送队列装填
     * @param _addr
     * @param config IDE | RTR
     */
    void Transmit(uint32_t _addr, uint8_t config = static_cast<uint8_t>(CAN_ID_HeaderTypeDef::STD) | static_cast<uint8_t>(CAN_RTR_HeaderTypeDef::DATA)) {
        txbuf.addr = _addr;
        txbuf.IDE = config & static_cast<uint8_t>(CAN_ID_HeaderTypeDef::EXT);
        txbuf.RTR = config & static_cast<uint8_t>(CAN_RTR_HeaderTypeDef::REMOTE);

        CAN_Base<ID>::GetInstance().Transmit(txbuf);
    }

    uint8_t &operator[](std::size_t index) {
        return txbuf.message[index];
    }

    uint8_t operator[](std::size_t index) const {
        return rxbuf[index];
    }

    uint32_t addr;
    uint8_t rxbuf[8] = {0};

private:
    CAN_Package_t txbuf = {8};
};

template<typename T = decltype(BSP_CANList[0])>
class FineMoteAux_CAN {
public:
    static void OnTxComplete(T hcan) {
        constexpr size_t maxID = CAN_BUS_MAXIMUM_COUNT;
        TxCompleteImpl<maxID>(hcan);
    }

    static void OnRxComplete(T hcan) {
        constexpr size_t maxID = CAN_BUS_MAXIMUM_COUNT;
        RxCompleteImpl<maxID>(hcan);
    }

private:
    template<size_t ID>
    static void TxCompleteImpl(T hcan) {
        if constexpr (BSP_CANList[ID] != nullptr) {
            if (hcan == BSP_CANList[ID]) {
                CAN_Base<ID>::GetInstance().TxHandle();
                return;
            }
        }
        if constexpr (ID > 1) {
            TxCompleteImpl<ID - 1>(hcan);
        }
    }

    template<size_t ID>
    static void RxCompleteImpl(T hcan) {
        if constexpr (BSP_CANList[ID] != nullptr) {
            if (hcan == BSP_CANList[ID]) {
                CAN_Base<ID>::GetInstance().RxHandle();
                return;
            }
        }
        if constexpr (ID > 1) {
            RxCompleteImpl<ID - 1>(hcan);
        }
    }
};

#endif
