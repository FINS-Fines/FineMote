#ifndef FINEMOTE_DSHOT_H
#define FINEMOTE_DSHOT_H

#define DSHOT_MIN_THROTTLE              (48)
#define DSHOT_MAX_THROTTLE              (2047)
#define DSHOT_3D_FORWARD_MIN_THROTTLE   (1048)
#define DSHOT_3D_BACKWARD_MAX_THROTTLE  (1047)
#define DSHOT_RANGE                     (1999)

#include "BSP_DSHOT.h"
#include "DeviceBase.h"
#include "etl/map.h"
#include "etl/vector.h"
#include "arm_math.h"


class DSHOT_Port : public DeviceBase {
private:
    struct DSHOT_GPIO_PortPins {
        GPIO_TypeDef *port;
        etl::map<uint16_t, uint16_t, 16> pins; // 使用map存储每个引脚的DshotPack
        DSHOT_GPIO_PortPins(GPIO_TypeDef *_port, uint16_t pin) : port(_port) {
            pins[pin] = 0;
        }
    };
    uint32_t outPutData[DSHOT_PACK_LENGTH * 3] {};
    etl::vector<DSHOT_GPIO_PortPins, 4> gpioPortsQueue = {};

    DSHOT_Port(){}

    void UpdateMiddleBit(etl::map<uint16_t, uint16_t, 16> &pins)
    {
        for (int symbol_index = DSHOT_PACK_HEAD_LENGTH; symbol_index < DSHOT_FRAME_LENGTH + DSHOT_PACK_HEAD_LENGTH; symbol_index++) {
            outPutData[symbol_index * 3 + 1] = 0;          // Reset bits are port dependent
        }
        for(auto &pin : pins) {
            uint16_t value = pin.second;
            uint32_t middleBitHigh = isInverted ? pin.first << 16 : pin.first;
            uint32_t middleBitLow = isInverted ? pin.first : pin.first << 16;
            for (int pos = DSHOT_PACK_HEAD_LENGTH; pos < DSHOT_FRAME_LENGTH + DSHOT_PACK_HEAD_LENGTH; pos++) {
                if (value & 0x8000) {
                    outPutData[pos * 3 + 1] |= middleBitHigh;
                }else {
                    outPutData[pos * 3 + 1] |= middleBitLow;
                }
                value <<= 1;
            }
        }
    }

    void GenerateMessage(etl::map<uint16_t, uint16_t, 16> &pins)
    {
        uint32_t portMask = 0;
        for(auto &pin : pins) {
            portMask |= pin.first;
        }
        uint32_t resetMask;
        uint32_t setMask;

        if (isInverted) {
            resetMask = portMask;
            setMask = (portMask << 16);
        } else {
            resetMask = (portMask << 16);
            setMask = portMask;
        }

        for(int hold_bit_index = 0; hold_bit_index < DSHOT_PACK_HEAD_LENGTH; hold_bit_index++) {
            outPutData[hold_bit_index * 3 + 0] = resetMask; // Always reset all ports
            outPutData[hold_bit_index * 3 + 1] = resetMask;
            outPutData[hold_bit_index * 3 + 2] = resetMask;
        }
        for (int symbol_index = DSHOT_PACK_HEAD_LENGTH; symbol_index < DSHOT_FRAME_LENGTH + DSHOT_PACK_HEAD_LENGTH; symbol_index++) {
            outPutData[symbol_index * 3 + 0] = setMask ; // Always set all ports
            outPutData[symbol_index * 3 + 1] = 0;          // Reset bits are port dependent
            outPutData[symbol_index * 3 + 2] = resetMask; // Always reset all ports
        }
        for(int hold_bit_index = DSHOT_FRAME_LENGTH + DSHOT_PACK_HEAD_LENGTH; hold_bit_index < DSHOT_PACK_LENGTH; hold_bit_index++) {
            outPutData[hold_bit_index * 3 + 0] = resetMask; // Always reset all ports
            outPutData[hold_bit_index * 3 + 1] = resetMask;
            outPutData[hold_bit_index * 3 + 2] = resetMask;
        }

        UpdateMiddleBit(pins);
    }

public:
    bool isInverted = false;

    DSHOT_Port(const DSHOT_Port&) = delete;
    DSHOT_Port& operator=(const DSHOT_Port&) = delete;

    static DSHOT_Port& GetInstance() {
        static DSHOT_Port instance;
        return instance;
    }

    void AddPin(GPIO_TypeDef *port, uint16_t pin) {
        for(auto &data : gpioPortsQueue) {
            if(data.port == port) {
                data.pins[pin] = 0;
                return;
            }
        }
        if(!gpioPortsQueue.full()) {
            gpioPortsQueue.push_back(DSHOT_GPIO_PortPins(port, pin));
        }
        else {
            // 如果队列已满，可以选择覆盖最旧的元素或抛出异常
            gpioPortsQueue[0] = DSHOT_GPIO_PortPins(port, pin);
        }
    }

    void SetDataPack(GPIO_TypeDef *port, uint16_t pin, uint16_t DshotDataPack) {
        for(auto &data : gpioPortsQueue) {
            if(data.port == port && data.pins.count(pin)) {
                data.pins[pin] = DshotDataPack;
                return;
            }
        }
    }

    void SendMessage(bool isReset = false) {
        static size_t currentIndex = 0;

        if (isReset) {
            currentIndex = 0; // 重新遍历队列
        }

        if (currentIndex >= gpioPortsQueue.size()) {
            return; // 队列已遍历完毕
        }

        GenerateMessage(gpioPortsQueue[currentIndex].pins);
        BSP_DSHOTs::GetInstance().TransmitMessage(outPutData, gpioPortsQueue[currentIndex].port, DSHOT_PACK_LENGTH * 3);
        currentIndex++;
    }

    void Handle() override {
        SendMessage(true);
    }
};



template <uint16_t ID>
class DSHOT_PIN {
public:
    DSHOT_PIN(const DSHOT_PIN &) = delete;
    DSHOT_PIN &operator=(const DSHOT_PIN &) = delete;
    static DSHOT_PIN &GetInstance() {
        static DSHOT_PIN instance;
        return instance;
    }

    void SetIntThrottle(int integerThrottle) {
        if(!isMotorEnabled) {
            targetThrottle = 0;
            GenerateDshotPack();
            return;
        }

        targetThrottle = integerThrottle;
        GenerateDshotPack();
    }

    void SetTargetThrottle(float throttle) {
        if(!isMotorEnabled) {
            targetThrottle = 0;
            GenerateDshotPack();
            return;
        }
        // range of throttle 3D(-1，1), normal(0，1)
        if(is3DModeEnabled) {
            if (fabsf(throttle) < 1e-6f) {
                targetThrottle = 0;
            }
            else {
                if(throttle < 0) {
                    targetThrottle = DSHOT_MIN_THROTTLE + static_cast<int>(roundf(fabsf(throttle) * (DSHOT_3D_BACKWARD_MAX_THROTTLE - DSHOT_MIN_THROTTLE)));
                    targetThrottle = targetThrottle < DSHOT_MIN_THROTTLE ? DSHOT_MIN_THROTTLE : targetThrottle;
                    targetThrottle = targetThrottle > DSHOT_3D_BACKWARD_MAX_THROTTLE ? DSHOT_3D_BACKWARD_MAX_THROTTLE : targetThrottle;
                }
                else {
                    targetThrottle = DSHOT_3D_FORWARD_MIN_THROTTLE + static_cast<int>(roundf(throttle * (DSHOT_MAX_THROTTLE - DSHOT_3D_FORWARD_MIN_THROTTLE)));
                    targetThrottle = targetThrottle < DSHOT_3D_FORWARD_MIN_THROTTLE ? DSHOT_3D_FORWARD_MIN_THROTTLE : targetThrottle;
                    targetThrottle = targetThrottle > DSHOT_MAX_THROTTLE ? DSHOT_MAX_THROTTLE : targetThrottle;
                }
            }
        }
        else {
            int integerThrottle = static_cast<int>(roundf(throttle * DSHOT_RANGE));
            targetThrottle = integerThrottle > DSHOT_RANGE ? DSHOT_MAX_THROTTLE : (integerThrottle <= 0 ? 0 : integerThrottle + DSHOT_MIN_THROTTLE); // 1-47保留为特殊指令
        }

        GenerateDshotPack();
    }

    void Set3DMode(bool isEnable) {
        is3DModeEnabled = isEnable;
    }

    void SetTelemetry(bool isEnable) {
        telemetryRequest = isEnable;
    }

    void EnableMotor() {
        isMotorEnabled = true;
    }

    void DisableMotor() {
        isMotorEnabled = false;
        targetThrottle = 0;
        GenerateDshotPack();
    }

private:
    uint16_t targetThrottle = 0;
    bool is3DModeEnabled = false;
    bool isInverted = false;
    bool telemetryRequest = false;
    bool isMotorEnabled = true; // 是否允许电机工作

    DSHOT_PIN() {
        BSP_DSHOT<ID>::GetInstance();
        DSHOT_Port::GetInstance().AddPin(BSP_DHOSTPortList[ID], BSP_DHOSTPinList[ID]);
        isInverted = DSHOT_Port::GetInstance().isInverted;
    }

    void GenerateDshotPack() {
        // 构建完整DShot数据包（11位油门 + 1位遥测 + 4位CRC）
        uint16_t packet{0};
        packet |= (targetThrottle << 5);
        packet |= (telemetryRequest ? 1<<4 : 0<<4);

        // 计算CRC并合并到数据包
        uint8_t crc = (packet >> 4) ^ (packet >> 8) ^ (packet >> 12);  // 取前12位计算
        if(isInverted) {
            crc = ~crc;
        }
        packet |= crc & 0x0F;

        DSHOT_Port::GetInstance().SetDataPack(BSP_DHOSTPortList[ID], BSP_DHOSTPinList[ID], packet);
    }
};

inline void DMA_XferCpltCallback(DMA_HandleTypeDef *hdma) {
    if (hdma->Instance == DSHOT_DMA_HANDLE.Instance) {
        DSHOT_Port::GetInstance().SendMessage();
    }
}


#endif
