// /*******************************************************************************
//  * Copyright (c) 2025.
//  * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
//  * All rights reserved.
//  ******************************************************************************/
//
// #ifndef FINEMOTE_SPI_BASE_HPP
// #define FINEMOTE_SPI_BASE_HPP
//
// #include "etl/map.h"
// #include "etl/queue.h"
// #include "BSP_SPI.h"
//
// #define SPI_MAP_SIZE 20
// #define SPI_TX_QUEUE_SIZE 16
//
//
// template<size_t ID>
// class SPI_Base {
// public:
//     static SPI_Base &GetInstance() {
//         static SPI_Base instance;
//         return instance;
//     }
//
//     SPI_Base(const SPI_Base &) = delete;
//
//     SPI_Base &operator=(const SPI_Base &) = delete;
//
//     void RxHandle() {
//     }
//
//     void TxHandle() {
//         if (!dataQueue.empty()) {
//             dataQueue.pop();
//         } else {
//             isTxComplete = true;
//         }
//     }
//
//     bool Transmit(SPI_Package_t &txbuf) {
//         if (dataQueue.full()) {
//             dataQueue.pop();
//         }
//         dataQueue.push(txbuf);
//         if (isTxComplete == true) {
//             TxHandle();
//             isTxComplete = false;
//         }
//         return true;
//     }
//
//     void BindRxBuffer(uint8_t *buffer, uint32_t addr) {
//         rxBufferMap[addr] = buffer;
//     }
//
// private:
//     etl::map<uint32_t, uint8_t *, SPI_MAP_SIZE> rxBufferMap;
//     etl::queue<std::pair<uint8_t *, uint16_t>,SPI_TX_QUEUE_SIZE> dataQueue;
//     bool isTxComplete = true;
//
//     SPI_Base() {
//         BSP_SPI<ID>::GetInstance();
//     }
// };
//
// template<size_t ID, size_t selectedID>
// class SPI_Agent {
// public:
//     explicit SPI_Agent(){
//         static_assert(ID > 0 && ID <= SPI_BUS_MAXIMUM_COUNT && BSP_SPIList[ID] != nullptr, "Using illegal SPI BUS");
//     }
//
//     void Transmit(uint8_t* data, uint16_t size) {
//     }
//
//     void TransmitReceive(uint8_t* txData, uint8_t* rxData, uint16_t size) {
//     }
// };
//
//
// template<typename T = decltype(BSP_SPIList[0])>
// class FineMoteAux_SPI {
// public:
//     static void OnTxComplete(T &instance) {
//         constexpr size_t maxID = sizeof(BSP_SPIList) / sizeof(BSP_SPIList[0]) - 1;
//         TxCompleteImpl<maxID>(instance);
//     }
//
//     static void OnRxComplete(T &instance, size_t size) {
//         constexpr size_t maxID = sizeof(BSP_SPIList) / sizeof(BSP_SPIList[0]) - 1;
//         RxCompleteImpl<maxID>(instance, size);
//     }
//
//     template<size_t ID>
//     static void TxCompleteImpl(T &instance) {
//         if constexpr (BSP_SPIList[ID] != nullptr) {
//             if (instance == BSP_SPIList[ID]) {
//                 SPI_Base<ID>::GetInstance().TxHandle();
//                 return;
//             }
//         }
//         if constexpr (ID > 1) {
//             TxCompleteImpl<ID - 1>(instance);
//         }
//     }
//
//     template<size_t ID>
//     static void RxCompleteImpl(T &instance, size_t size) {
//         if constexpr (BSP_SPIList[ID] != nullptr) {
//             if (instance == BSP_SPIList[ID]) {
//                 SPI_Base<ID>::GetInstance().RxHandle(size);
//                 return;
//             }
//         }
//         if constexpr (ID > 1) {
//             RxCompleteImpl<ID - 1>(instance, size);
//         }
//     }
// };
//
// #endif
