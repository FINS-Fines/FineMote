/******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_SPI_BASE_HPP
#define FINEMOTE_SPI_BASE_HPP

#include <cstdint>
#include <cstddef>
#include "etl/queue.h"
#include "BSP_SPI.h"

template<size_t ID>
class SPI_Agent;

// ============ SPI_Base ============
template<size_t ID>
class SPI_Base {
public:
    static SPI_Base& GetInstance() {
        static SPI_Base instance;
        return instance;
    }
    SPI_Base(const SPI_Base&)            = delete;
    SPI_Base& operator=(const SPI_Base&) = delete;

    void Transmit(const SPI_Agent<ID>& agent) {
        Enqueue(agent);
    }
    void Receive(const SPI_Agent<ID>& agent) {
        Enqueue(agent);
    }
    void TransmitReceive(const SPI_Agent<ID>& agent) {
        Enqueue(agent);
    }

    void TxRxHandle() {
        BSP_SPI<ID>::CS_High(dataQueue.front()->deviceID);
        isComplete = true;
        dataQueue.pop();
        ProcessQueue();
    }

private:
    etl::queue<const SPI_Agent<ID>*, BSP_SPI<ID>::GET_QUEUE_SIZE()> dataQueue;
    bool isComplete = true;

    SPI_Base() {
        BSP_SPI<ID>::GetInstance();
    }

    void Enqueue(const SPI_Agent<ID>& agent) {
        if (dataQueue.full()) {
            dataQueue.pop();
        }
        dataQueue.emplace(&agent);
        if (isComplete) {
            ProcessQueue();
        }
    }

    void ProcessQueue() {
        if (dataQueue.empty()) {
            return;
        }
        isComplete = false;
        const SPI_Agent<ID>* const agent = dataQueue.front();
        BSP_SPI<ID>::CS_Low(agent->deviceID);
        if (agent->dir == SPI_Agent<ID>::Direction::Tx) {
            BSP_SPI<ID>::GetInstance().Transmit(
                agent->txData,
                agent->size
            );
        } else if (agent->dir == SPI_Agent<ID>::Direction::Rx) {
            BSP_SPI<ID>::GetInstance().Receive(
                agent->rxBuffer,
                agent->size
            );
        } else {
            BSP_SPI<ID>::GetInstance().TransmitReceive(
                agent->txData,
                agent->rxBuffer,
                agent->size
            );
        }
    }
};

// ============ SPI_Agent ============
template<size_t ID>
class SPI_Agent {
public:
    enum class Direction : uint8_t {
        Tx,
        Rx,
        TxRx,
    };

    template<size_t N>
    explicit SPI_Agent(const uint8_t& deviceID, uint8_t (&buffer)[N])
        : deviceID(deviceID),
          rxBuffer(buffer),
          rxBufferSize(N)
    {
        static_assert(
            ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr,
            "Using illegal SPI BUS"
        );
    }

    explicit SPI_Agent(const uint8_t& deviceID)
        : deviceID(deviceID),
          rxBuffer(nullptr),
          rxBufferSize(0)
    {
        static_assert(
            ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr,
            "Using illegal SPI BUS"
        );
    }

    void Transmit(uint8_t* const& pTxData, const size_t& size) {
        this->txData = pTxData;
        this->size   = size;
        this->dir    = Direction::Tx;
        SPI_Base<ID>::GetInstance().Transmit(*this);
    }

    void Receive(const size_t& size) {
        this->size = size > rxBufferSize ? rxBufferSize : size;
        this->dir  = Direction::Rx;
        SPI_Base<ID>::GetInstance().Receive(*this);
    }

    void TransmitReceive(uint8_t* const& pTxData, const size_t& size) {
        if (size > rxBufferSize) {
            return;
        }
        this->txData = pTxData;
        this->size   = size;
        this->dir    = Direction::TxRx;
        SPI_Base<ID>::GetInstance().TransmitReceive(*this);
    }

private:
    friend class SPI_Base<ID>;

    const uint8_t  deviceID;
    uint8_t* const rxBuffer;
    const size_t   rxBufferSize;

    uint8_t* txData;
    Direction dir;
    size_t   size;
};

// ============ FineMoteAux_SPI ============
template<typename T = decltype(BSP_SPIList[0])>
class FineMoteAux_SPI {
public:
    static void OnTxRxComplete(const T& hspi) {
        constexpr size_t maxID = sizeof(BSP_SPIList) / sizeof(BSP_SPIList[0]) - 1;
        TxRxCompleteImpl<maxID>(hspi);
    }

private:
    template<size_t ID>
    static void TxRxCompleteImpl(const T& hspi) {
        if constexpr (BSP_SPIList[ID] != nullptr) {
            if (hspi == BSP_SPIList[ID]) {
                SPI_Base<ID>::GetInstance().TxRxHandle();
                return;
            }
        }
        if constexpr (ID > 1) {
            TxRxCompleteImpl<ID - 1>(hspi);
        }
    }
};

#endif