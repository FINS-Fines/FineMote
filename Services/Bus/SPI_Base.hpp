/******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#ifndef FINEMOTE_SPI_BASE_HPP
#define FINEMOTE_SPI_BASE_HPP

#include "BSP_SPI.h"
#include <etl/queue.h>
#include <cstdint>
#include <cstddef>

#define SPI_QUEUE_SIZE 16
#define SPI_AGENT_RX_SIZE 16

enum class SPI_Direction : uint8_t {
    Tx,
    Rx,
    TxRx,
};

typedef struct {
    uint8_t deviceID;
    SPI_Direction dir;
    uint8_t *tx;
    uint8_t *rx;
    uint16_t size;
} SPI_Package_t;

template<size_t ID>
class SPI_Base {
public:
    static SPI_Base& GetInstance() {
        static SPI_Base instance;
        return instance;
    }

    SPI_Base(const SPI_Base&) = delete;
    SPI_Base& operator=(const SPI_Base&) = delete;

    void Enqueue(const SPI_Package_t& pkg) {
        if (dataQueue.full()) {
            dataQueue.pop();
        }
        dataQueue.push(pkg);
        if (isComplete) {
            ProcessQueue();
        }
    }

    void TxRxHandle()
    {
        BSP_SPI<ID>::CS_High(SPI_CSList[currentPkg.deviceID]);
        isComplete = true;
        ProcessQueue();
    }

private:
    etl::queue<SPI_Package_t, SPI_QUEUE_SIZE> dataQueue;
    SPI_Package_t currentPkg = {};
    bool isComplete = true;

    SPI_Base() {
        BSP_SPI<ID>::GetInstance();
    }

    void ProcessQueue() {
        if (!isComplete) return;
        if (dataQueue.empty()) return;

        isComplete = false;
        currentPkg = dataQueue.front();
        dataQueue.pop();

        BSP_SPI<ID>::CS_Low(SPI_CSList[currentPkg.deviceID]);

        if (currentPkg.dir == SPI_Direction::Tx) {
            BSP_SPI<ID>::GetInstance().Transmit(currentPkg.tx, currentPkg.size);
        } else if (currentPkg.dir == SPI_Direction::Rx) {
            BSP_SPI<ID>::GetInstance().Receive(currentPkg.rx, currentPkg.size);
        } else {
            BSP_SPI<ID>::GetInstance().TransmitReceive(currentPkg.tx, currentPkg.rx, currentPkg.size);
        }
    }
};

/* 考虑到SPI通信的特点，无论是发送还是接收，都需要由主设备主动发出
 * 因此，Agent提供了Transmit和Receive的接口
 * 无论收发，都需要主动调用
 */

template<size_t ID>
class SPI_Agent {
public:
    explicit SPI_Agent(uint8_t deviceID) : deviceID(deviceID) {
        static_assert(ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr, "Using illegal SPI BUS");
    }

    void Transmit(uint8_t* pTxData, uint16_t size) {
        if (pTxData == nullptr) return; // 确保pTxData不为空指针
        SPI_Package_t pkg = {};
        pkg.deviceID = deviceID;
        pkg.dir = SPI_Direction::Tx;
        pkg.tx = pTxData;
        pkg.rx = nullptr;
        pkg.size    = (size > SPI_AGENT_RX_SIZE) ? SPI_AGENT_RX_SIZE : size;
        SPI_Base<ID>::GetInstance().Enqueue(pkg);
    }

    void Receive(uint8_t* pRxData, uint16_t size) {
        SPI_Package_t pkg = {};
        pkg.deviceID = deviceID;
        pkg.dir = SPI_Direction::Rx;
        pkg.tx = nullptr;
        pkg.rx = pRxData;
        pkg.size    = (size > SPI_AGENT_RX_SIZE) ? SPI_AGENT_RX_SIZE : size;
        SPI_Base<ID>::GetInstance().Enqueue(pkg);
    }

    void TransmitReceive(uint8_t* pTxData, uint8_t* pRxData, uint16_t size) {
        if (pTxData == nullptr) return; // 确保pTxData不为空指针
        SPI_Package_t pkg = {};
        pkg.deviceID = deviceID;
        pkg.dir = SPI_Direction::TxRx;
        pkg.tx = pTxData;
        pkg.rx = pRxData;
        pkg.size    = (size > SPI_AGENT_RX_SIZE) ? SPI_AGENT_RX_SIZE : size;
        SPI_Base<ID>::GetInstance().Enqueue(pkg);
    }

    // uint8_t operator[](std::size_t index) const {
    //     return rxbuf[index];
    // }
private:
    uint8_t deviceID;
    // uint8_t rxbuf[SPI_AGENT_RX_SIZE] = {0};
};

template<typename T = decltype(BSP_SPIList[0])>
class FineMoteAux_SPI {
public:
    static void OnTxRxComplete(T hspi) {
        constexpr size_t maxID = sizeof(BSP_SPIList) / sizeof(BSP_SPIList[0]) - 1;
        TxRxCompleteImpl<maxID>(hspi);
    }

private:
    template<size_t ID>
    static void TxRxCompleteImpl(T hspi) {
        if constexpr (BSP_SPIList[ID] != nullptr) {
            if (hspi == BSP_SPIList[ID]) {
                SPI_Base<ID>::GetInstance().TxRxHandle();
                return;
            }
        }
        if constexpr (ID > 1) TxRxCompleteImpl<ID - 1>(hspi);
    }
};

#endif