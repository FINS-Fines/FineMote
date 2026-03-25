/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MANIPULATOR_HPP
#define FINEMOTE_MANIPULATOR_HPP

#include "Control/ImplementControlBase.hpp"
#include "EncoderBase.hpp"
#include "CRC.h"

#define MANIPULATOR_PAYLOAD_LENGTH 29

template<size_t N>
auto createAmplifiers() -> std::array<Amplifier<1>, N> {
    return {};
}

enum class EndEffectorState : uint8_t { OPEN = 0, CLOSE = 1 };

struct ManipulatorAngle{  // 单位为弧度
    float angleA{0};
    float angleB{0};
    float angleC{0};
    float angleD{0};
    float angleE{0};
    float angleF{0};
}__packed manipulator_angle;

/**
 * @brief UART5 的解码回调函数，用于机械臂关节数据接收
 */
static uint16_t uart_cnt{};

std::function<void(uint8_t *, size_t)> DecodeManipulatorFrame = [](uint8_t* data, size_t length){
    // 帧格式: [Header:0xAA] [dataID] [Length] [Payload...] [CRC] [Trailer:0xBB]
    if (length < 5)
        return;

    // 验证帧头、帧尾、dataID
    if(data[0] != 0xAA || data[length - 1] != 0xBB || data[1] != 0x0B){
        return;
    }
    uint8_t payloadLen = data[2];

    // 验证帧长度: 总长 = 1(Header) + 1(dataID) + 1(Length) + payloadLen + 1(CRC) + 1(Trailer)
    if(length != (size_t)(payloadLen + 5)){
        return;
    }

    uint8_t crcPos = 3 + payloadLen;
    uint8_t expectedCRC = CRC8Calc(data + 3, payloadLen);
    if(data[crcPos] != expectedCRC){
        return;
    }

    memcpy(&manipulator_angle, data + 3, payloadLen);

    uart_cnt++;
};

template<size_t ID, size_t N>
class ManipulatorUARTReceiver {
public:
    ManipulatorUARTReceiver() : buffer(DecodeManipulatorFrame) {}

private:
    UARTBuffer<ID, N> buffer;
};


class Manipulator : public DeviceBase{
public:
    bool isInitFinished = false;
    bool GetInitCommand = false;

    Manipulator(MotorBase* motorA, MotorBase* motorB, MotorBase* motorC, MotorBase* motorD, MotorBase* motorE,
                EncoderBase* encoderA, EncoderBase* encoderB, EncoderBase* encoderC, EncoderBase* encoderD):
            _motorA(motorA), _motorB(motorB), _motorC(motorC), _motorD(motorD), _motorE(motorE),
            _encoderA(encoderA), _encoderB(encoderB), _encoderC(encoderC), _encoderD(encoderD){}

    void SetAngle(float angleA, float angleB, float angleC, float angleD, float angleE) {    // 单位为角度
        targetAngle[0] = angleA;
        targetAngle[1] = angleB;
        targetAngle[2] = -angleC;
        targetAngle[3] = angleD;
        targetAngle[4] = -angleE;
    }

    // void SetEndEffectorAngle(const bool isOpen) {
    //     endEffectorState = isOpen;
    // }

    // 设置多圈目标角度
    void SendAngle() {
        if(isInitFinished) {
            _motorA->SetTargetAngle(targetAngle[0]);
            _motorB->SetTargetAngle(targetAngle[1]);
            _motorC->SetTargetAngle(targetAngle[2]);
            _motorD->SetTargetAngle(targetAngle[3]);
            _motorE->SetTargetAngle(targetAngle[4]);
        }
    }

    void Handle() override {
        if(!isInitFinished) {
            UpdateEncoderData();
            ManipulatorInit();
        }
        SendAngle();
    }


private:
    MotorBase* _motorA{};
    MotorBase* _motorB{};
    MotorBase* _motorC{};
    MotorBase* _motorD{};
    MotorBase* _motorE{};


    EncoderBase* _encoderA{};
    EncoderBase* _encoderB{};
    EncoderBase* _encoderC{};
    EncoderBase* _encoderD{};

    float _encoderAngleA{0};
    float _encoderAngleB{0};
    float _encoderAngleC{0};
    float _encoderAngleD{0};



    // bool endEffectorState = END_EFFECTOR_OPEN;

    float targetAngle[5] = {0, 0, 0, 0, 0};
    const float reductionRatio[5] = {1, 1, 1, 1, 1};  // 减速比

    void UpdateEncoderData() {
        _encoderAngleA = _encoderA->getPosition();
        _encoderAngleB = _encoderB->getPosition();
        _encoderAngleC = _encoderC->getPosition();
        _encoderAngleD = _encoderD->getPosition();
    }

    void ManipulatorInit() {
        if(GetInitCommand) {
            // Todo: 初始化电机和编码器
            isInitFinished = true;
            GetInitCommand = false;
        }
    }


    // Todo: 状态机
};

#endif //FINEMOTE_MANIPULATOR_HPP
