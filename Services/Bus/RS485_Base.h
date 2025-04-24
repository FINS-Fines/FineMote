#ifndef FINEMOTE_RS485_BASE_H
#define FINEMOTE_RS485_BASE_H

#include "FixedSizeQueue.hpp"
#include "FixedSizeMap.hpp"
#include "DoubleBuff.hpp"
#include "ProjectConfig.h"

#ifdef RS485_BASE_MODULE

#include "UARTBaseLite.h"
#include "FixedSizeQueue.hpp"

#define RS485_RX_BUF_SIZE (50) // 接收缓冲区大小
#define RS485_TX_BUF_SIZE (50) // 发送缓冲区大小
#define RS485_TX_QUE_LENG (10) // 发送缓冲区大小
#define RS485_AGENT_NUM (10) // RS485设备数量

template <uint8_t busID>
class RS485_Agent;

template <uint8_t busID>
class RS485_Base :public DeviceBase{
public:
    uint8_t rxBuffer[RS485_RX_BUF_SIZE]{0};
    FixedSizeQueue<std::pair<uint8_t*, uint16_t>, RS485_TX_QUE_LENG> txQueue;
    FixedSizeMap<uint8_t, RS485_Agent<busID>*, RS485_AGENT_NUM> agentMap;

    explicit RS485_Base(){
        // 确保RS485总线的初始化
        HALInit::GetInstance();

        // 启动UART接收中断。最多接收200个字节
        HAL_UARTEx_ReceiveToIdle_IT(
            uartHandleList[busID],
            UARTBaseLite<busID>::GetInstance().rxBuffer[0],
            200);

        // 设置接收回调函数
        std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
            RS485_Base::GetInstance().Decode(data, length);
        };

        // 绑定接收回调函数
        UARTBaseLite<busID>::GetInstance().Bind(decodeFunc);
    }

    void Decode(uint8_t* data, uint16_t size){
        memcpy(&rxBuffer, data, size);
    }

    static RS485_Base& GetInstance() {
        static RS485_Base instance;
        return instance;
    }

    void Transmit(uint8_t* data, uint16_t size) {
        txQueue.enqueue(std::make_pair(data, size));
    }

    void TxLoader() {
        if (!txQueue.isEmpty()) {
#ifndef RS485_NO_XXX
            HAL_GPIO_WritePin(rs485TxPortList[busID], rs485TxPinList[busID], GPIO_PIN_SET);
#endif
            // 发送数据
            HAL_UART_Transmit_IT(uartHandleList[busID], txQueue.frontElement().first, txQueue.frontElement().second);
            txQueue.dequeue();
        }
    }

    void RxHandle() {
#ifndef RS485_NO_XXX
        // 设置为接收模式
        HAL_GPIO_WritePin(rs485TxPortList[busID], rs485TxPinList[busID], GPIO_PIN_RESET);
#endif

        // 将数据拷贝给 agent
        uint8_t dev_addr = rxBuffer[0];
        RS485_Agent<busID>* agent_p = nullptr;
        if (agentMap.fetch(dev_addr, agent_p))
            if (agent_p != nullptr)
                std::copy(rxBuffer, rxBuffer + RS485_RX_BUF_SIZE, agent_p->rxbuf);
    }

    void Handle() final {
        TxLoader();
    }
};

template <uint8_t busID>
class RS485_Agent : public DeviceBase {
public:
    explicit RS485_Agent(uint32_t _addr) : addr(_addr) {
        // 构造将 agent 注册到 RS485_Base
        RS485_Base<busID>::GetInstance().agentMap.insert(addr, this);
    }

    void SendMsg(uint16_t size){
        // 发送数据
        RS485_Base<busID>::GetInstance().Transmit(txbuf,size);
    }

    /**** 验证地址（貌似放这里不太合适） ****/
    void Handle() override{}

    uint32_t addr;
    uint8_t rxbuf[RS485_RX_BUF_SIZE]{0};
    uint8_t txbuf[RS485_TX_BUF_SIZE]{0};
};

#endif //RS485_BASE_MODULE

#endif //FINEMOTE_RS485_BASE_H