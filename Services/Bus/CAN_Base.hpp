/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CAN_BASE_HPP
#define FINEMOTE_CAN_BASE_HPP

#include "etl/flat_map.h"
#include "etl/queue.h"

#include "Bus/CAN_Types.hpp"

template<uint8_t ID>
class BSP_CAN; //前置声明BSP_CAN

#define CAN_MAP_SIZE 20
#define CAN_TX_QUEUE_SIZE 16

/**
 * Todo:
 * 远程帧处理
 */

template<uint8_t ID>
class CAN_Base {
    using HardwareType = typename CAN_Traits<ID>::Type;
    using PackageType = CAN_Package<HardwareType>;

    struct RxNode {
        uint8_t* ptr;
        uint8_t size;
    };

public:
    static CAN_Base& GetInstance() {
        static CAN_Base instance;
        return instance;
    }

    CAN_Base(const CAN_Base&) = delete;
    CAN_Base& operator=(const CAN_Base&) = delete;

    void RxHandle() {
        PackageType package;

        BSP_CAN<ID>::GetInstance().Receive(package);

        if (auto it = rxBufferMap.find(package.id); it != rxBufferMap.end()) {
            if (const auto& node = it->second; package.len <= node.size) {
                memcpy(node.ptr, package.data, package.len);
            }
        }
    }

    void TxHandle() {
        if (!dataQueue.empty()) {
            PackageType& package = dataQueue.front();
            BSP_CAN<ID>::GetInstance().Transmit(package);
            dataQueue.pop();
        } else {
            isTxComplete = true;
        }
    }

    bool Transmit(PackageType& tx_buf) {
        if (dataQueue.full()) {
            dataQueue.pop();
        }
        dataQueue.push(tx_buf);
        if (isTxComplete == true) {
            TxHandle();
            isTxComplete = false;
        }
        return true;
    }

    void BindRxBuffer(uint8_t* buffer, const uint8_t size, uint32_t addr) {
        rxBufferMap[addr] = { buffer, size };
    }

private:
    etl::flat_map<uint32_t, RxNode, CAN_MAP_SIZE> rxBufferMap;
    etl::queue<PackageType, CAN_TX_QUEUE_SIZE> dataQueue;
    bool isTxComplete = true;

    CAN_Base() {
        BSP_CAN<ID>::GetInstance();
    }
};

template<uint8_t ID, typename AgentType = BxCAN>
class CAN_Agent {
    using HardwareType = typename CAN_Traits<ID>::CAN_Type;
    static_assert(
        !(etl::is_same_v<HardwareType, BxCAN> && etl::is_same_v<AgentType, FDCAN>),
        "Fatal: Cannot use FDCAN agent on BxCAN hardware!"
    );

    static_assert(ID > 0 && ID <= CAN_BUS_MAXIMUM_COUNT && BSP_CANList[ID] != nullptr, "Using illegal CAN BUS");

public:
    explicit CAN_Agent(uint32_t addr): addr_(addr) {
        CAN_Base<ID>::GetInstance().BindRxBuffer(rx_buf_, RxBufferSize, addr);
    }

    void SetLength(uint8_t _len) {
        if (_len > RxBufferSize) {
            _len = RxBufferSize;
        }
        tx_buf_.len = _len;
    }

    /**
     * @brief CAN发送队列装填
     * @param addr
     * @param config IDE | RTR
     * @param IDE CAN_ID_STD or CAN_ID_EXT
     * @param RTR CAN_RTR_DATA or CAN_RTR_REMOTE
     */
    template<typename = void>
    etl::enable_if_t<etl::is_same_v<AgentType, BxCAN>, void>
    Transmit(const uint32_t addr, const uint8_t config = CAN_ID_STD | CAN_RTR_DATA) {
        tx_buf_.addr = addr;
        tx_buf_.flags.is_ext = (config & CAN_ID_EXT) ? 1 : 0;
        tx_buf_.flags.is_rtr = (config & CAN_RTR_REMOTE) ? 1 : 0;

        if constexpr (etl::is_same_v<HardwareType, FDCAN>) {
            tx_buf_.flags.is_fd = false;
            tx_buf_.flags.is_brs = false;
        }

        CAN_Base<ID>::GetInstance().Transmit(tx_buf_);
    }

    template<typename = void>
    etl::enable_if_t<std::is_same_v<AgentType, FDCAN>, void>
    Transmit(const uint32_t addr, const bool is_brs = true, const uint8_t config = CAN_ID_STD) {
        tx_buf_.addr = addr;

        tx_buf_.flags.is_ext = (config & CAN_ID_EXT) ? 1 : 0;
        tx_buf_.flags.is_rtr = 0;

        tx_buf_.flags.is_fd = true;
        tx_buf_.flags.is_brs = is_brs ? 1 : 0;

        CAN_Base<ID>::GetInstance().Transmit(tx_buf_);
    }

    uint8_t& operator[](std::size_t index) {
        return tx_buf_.data[index];
    }

    uint8_t operator[](std::size_t index) const {
        return rx_buf_[index];
    }

    uint32_t addr_;

    static constexpr std::size_t RxBufferSize = CAN_Package<AgentType>::BufferSize;
    uint8_t rx_buf_[RxBufferSize] {};

private:
    CAN_Package<HardwareType> tx_buf_ {};
};

template<typename T = decltype(BSP_CANList[0])>
class FineMoteAux_CAN {
public:
    static void OnTxComplete(T hcan) {
        constexpr std::size_t maxID = std::size(BSP_CANList) - 1;
        TxCompleteImpl<maxID>(hcan);
    }

    static void OnRxComplete(T hcan) {
        constexpr std::size_t maxID = std::size(BSP_CANList) - 1;
        RxCompleteImpl<maxID>(hcan);
    }

private:
    template<uint8_t ID>
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

    template<uint8_t ID>
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
