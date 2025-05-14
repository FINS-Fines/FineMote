#ifndef DSHOT_H
#define DSHOT_H

#define PACK_LENGTH 20
#define DSHOT_FRAME_LENGTH 16
#define RX_LENGTH 100

#include "../../BSP/Robomaster_C/Drivers/STM32F4xx_HAL_Driver/Inc/stm32f4xx_ll_gpio.h"
#include "DeviceBase.h"
#include "ProjectConfig.h"

extern DMA_HandleTypeDef hdma_tim8_up;
extern DMA_HandleTypeDef hdma_tim1_up;

class DSHOT : public DeviceBase{
private:
    uint32_t outPutData[PACK_LENGTH * 3] {};
    uint8_t GPIOC_DSHOT_PIN_FLAG[16] {0,0,0,0,0,0,1,1,1,1,0,0,0,0,0,0};
    uint16_t targetThrottle{};
    uint16_t rxBuffer[RX_LENGTH]{};
    uint8_t pin6Data[RX_LENGTH]{};
    uint8_t pin7Data[RX_LENGTH]{};
    uint8_t pin8Data[RX_LENGTH]{};
    uint8_t pin9Data[RX_LENGTH]{};
    uint8_t motorPolePair = 7; // 电机极对数
    bool telemetryRequest;
    bool inverted;

    float motorRPM1{};
    float motorRPM2{};
    float motorRPM3{};
    float motorRPM4{};

    uint8_t GCRToData(uint8_t encoded) {
        const uint8_t gcr_table[32] = {
            0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF,
            0xFF, 0x9,  0xA,  0xB,  0xFF, 0xD,  0xE,  0xF,
            0xFF, 0x2,  0x3,  0xFF, 0x5,  0x6,  0x7,  0xFF,
            0x0,  0x8,  0x1,  0xFF, 0x4,  0xC,  0xFF, 0xFF
        };
        if (encoded < 32) {
            uint8_t data = gcr_table[encoded];
            if (data != 0xFF) return data;
        }
        return 0xFF; // 无效编码
    }

public:
    bool isTxFinished = true;

    DSHOT(bool _telemetryRequest = true, bool _inverted = true):telemetryRequest(_telemetryRequest),inverted(_inverted) {
        Init(0x03c0);
    }

    void StartTransmit() {
        __HAL_TIM_ENABLE_DMA(&htim8, TIM_DMA_UPDATE);
        HAL_TIM_Base_Start(&htim8);
        // HAL_DMA_Start(&hdma_tim8_up, (uint32_t)outPutData, (uint32_t)&GPIOC->BSRR, PACK_LENGTH * 3);
        HAL_DMA_Start_IT(&hdma_tim8_up, (uint32_t)outPutData, (uint32_t)&GPIOC->BSRR, PACK_LENGTH * 3);
    }

    void InitReceive() {
        __HAL_TIM_ENABLE_DMA(&htim1, TIM_DMA_UPDATE);
        HAL_TIM_Base_Start(&htim1);
        // HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)&GPIOC->IDR, (uint32_t)rxBuffer, RX_LENGTH);
    }

    void Receive() {
        HAL_DMA_Start_IT(&hdma_tim1_up, (uint32_t)&GPIOC->IDR, (uint32_t)rxBuffer, RX_LENGTH);
    }

    void Decode() {
        for (int i = 0; i < RX_LENGTH; ++i) {
            pin6Data[i] = (rxBuffer[i] & (1 << 6)) ? 1 : 0;
            pin7Data[i] = (rxBuffer[i] & (1 << 7)) ? 1 : 0;
            pin8Data[i] = (rxBuffer[i] & (1 << 8)) ? 1 : 0;
            pin9Data[i] = (rxBuffer[i] & (1 << 9)) ? 1 : 0;
        }

        float rpm = CalaVel(pin6Data);
        rpm = CalaVel(pin6Data) < 0 ? motorRPM1:rpm;
        motorRPM1 = rpm;

        rpm = CalaVel(pin7Data);
        rpm = CalaVel(pin7Data) < 0 ? motorRPM2:rpm;
        motorRPM2 = rpm;

        rpm = CalaVel(pin8Data);
        rpm = CalaVel(pin8Data) < 0 ? motorRPM3:rpm;
        motorRPM3 = rpm;

        rpm = CalaVel(pin9Data);
        rpm = CalaVel(pin9Data) < 0 ? motorRPM4:rpm;
        motorRPM4 = rpm;
    }

    float CalaVel(uint8_t* buffer) {
        uint8_t GCRData[20]{};
        uint16_t positon{0};

        while (buffer[positon] != 0) {
            positon++;
        }

        for(int i = 0; i < 20;) {
            uint8_t length = FindEdge(&buffer[positon]);
            positon += length;
            if(length <= 3) {
                GCRData[i] = 1;
                i++;
            }
            else if(length > 3 && length <= 6) {
                GCRData[i] = 0;
                GCRData[i + 1] = 1;
                i += 2;
            }
            else if(length > 6 && length <= 9){
                GCRData[i] = 0;
                GCRData[i + 1] = 0;
                GCRData[i + 2] = 1;
                i += 3;
            }
            else {
                switch (i) {
                case 18:
                    GCRData[i] = 0;
                    GCRData[i + 1] = 0;
                    i = 20;
                    break;
                case 19:
                    GCRData[i] = 0;
                    i = 20;
                    break;
                default:
                    return -1;//表示出错
                }
            }
        }

        // 将20位GCR编码转换为16位数据
        uint16_t decodedData = 0;
        for (int seg = 0; seg < 4; seg++) {
            uint8_t encodedByte = 0;
            for (int bit = 0; bit < 5; bit++) {
                encodedByte = (encodedByte << 1) | GCRData[seg * 5 + bit];
            }
            uint8_t data = GCRToData(encodedByte);
            if (data == 0xFF) return -1; // 无效编码
            decodedData = (decodedData << 4) | data;
        }

        uint8_t crc = (decodedData >> 4) ^ (decodedData >> 8) ^ (decodedData >> 12);
        crc = ~crc; //取反码
        // if((crc & 0xff) == (decodedData & 0xff))
        {
            if(decodedData >> 4 == 0x0fff) {
                return 0; //默认速度0
            }
            float RPM{0};
            uint8_t shiftLeftBits = decodedData >> 13;
            RPM = static_cast<float>(((decodedData >> 4) & 0b111111111) << shiftLeftBits);
            if(!RPM) {
                return -1;
            }
            RPM = (1000000 / RPM * 60 + 50) / static_cast<float>(motorPolePair);
            return RPM;
        }
        return -1;
    }

    uint8_t FindEdge(uint8_t* buffer) {
        uint8_t i = 0;
        while(buffer[i] == buffer[0]) {
            if(i > 9){return 0xff;} //在GCR编码中不可能采样到两个以上0
            i++;
        }
        return i; //返回间隔点数
    }

    void SetTargrtThrottle(uint16_t target) {
        if(target > 2047) target = 2047;
        else if(target < 48) target = 0; // 保留1-47特殊指令
        targetThrottle = target;
    }

    uint8_t CRC4Calc(uint16_t data) {
        uint8_t crc = 0;
        data <<= 1; // 腾出CRC位
        for(int i=11; i>=0; --i){
            bool bit = (data >> (15 - i)) & 0x1;
            crc ^= (bit << 3);
            if(crc & 0x8) crc ^= 0x13; // DShot多项式
            crc = (crc << 1) & 0xF;
        }
        return crc;
    }

    void Init(uint16_t portMask)
    {
        uint32_t resetMask;
        uint32_t setMask;

        if (inverted) {
            resetMask = portMask;
            setMask = (portMask << 16);
        } else {
            resetMask = (portMask << 16);
            setMask = portMask;
        }

        for (int symbol_index = 0; symbol_index < DSHOT_FRAME_LENGTH; symbol_index++) {
            outPutData[symbol_index * 3 + 0] |= setMask ; // Always set all ports
            outPutData[symbol_index * 3 + 1] = 0;          // Reset bits are port dependent
            outPutData[symbol_index * 3 + 2] |= resetMask; // Always reset all ports
        }

        for(int hold_bit_index = DSHOT_FRAME_LENGTH; hold_bit_index < PACK_LENGTH; hold_bit_index++) {
            outPutData[hold_bit_index * 3 + 0] |= resetMask; // Always reset all ports
            outPutData[hold_bit_index * 3 + 1] |= resetMask;
            outPutData[hold_bit_index * 3 + 2] |= resetMask;
        }
    }

    void OutputDataSet(int pinNumber, uint16_t value)
    {
        uint32_t middleBitLow;
        uint32_t middleBitHigh;
        if (inverted) {
            middleBitLow = (1 << (pinNumber + 0));
            middleBitHigh = (1 << (pinNumber + 16));
        } else {
            middleBitLow = (1 << (pinNumber + 16));
            middleBitHigh = (1 << (pinNumber + 0));
        }

        for (int pos = 0; pos < 16; pos++) {
            if (value & 0x8000) {
                outPutData[pos * 3 + 1] |= middleBitHigh;
            }else {
                outPutData[pos * 3 + 1] |= middleBitLow;
            }
            value <<= 1;
        }
    }

    void ResetMiddleBits() {
        for (int symbol_index = 0; symbol_index < DSHOT_FRAME_LENGTH; symbol_index++) {
            outPutData[symbol_index * 3 + 1] = 0;
        }
    }

    void GenerateMsg() {
        // 构建完整DShot数据包（11位油门 + 1位遥测 + 4位CRC）
        uint16_t packet{0};
        packet |= (targetThrottle << 5);
        packet |= (telemetryRequest ? 1<<4 : 0<<4);

        // 计算CRC并合并到数据包
        uint8_t crc = (packet >> 4) ^ (packet >> 8) ^ (packet >> 12);  // 取前12位计算
        if(inverted) {
            crc = ~crc;
        }
        packet |= crc & 0x0F;

        ResetMiddleBits();

        OutputDataSet(6,packet);
        OutputDataSet(7,packet);
        OutputDataSet(8,packet);
        OutputDataSet(9,packet);
    }

    void Switch2Receive() {
        LL_GPIO_SetPinMode(GPIOC,DSHOT6_Pin,LL_GPIO_MODE_INPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT7_Pin,LL_GPIO_MODE_INPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT8_Pin,LL_GPIO_MODE_INPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT9_Pin,LL_GPIO_MODE_INPUT);
    }

    void Switch2Transmit() {
        LL_GPIO_SetPinMode(GPIOC,DSHOT6_Pin,LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT7_Pin,LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT8_Pin,LL_GPIO_MODE_OUTPUT);
        LL_GPIO_SetPinMode(GPIOC,DSHOT9_Pin,LL_GPIO_MODE_OUTPUT);
    }

    void Handle() override {
        Switch2Transmit();
        GenerateMsg();
        HAL_DMA_Start_IT(&hdma_tim8_up, (uint32_t)outPutData, (uint32_t)&GPIOC->BSRR, PACK_LENGTH * 3);
    }
};


#endif // DSHOT_H
