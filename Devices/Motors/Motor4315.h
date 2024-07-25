/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MOTOR4315_H
#define FINEMOTE_MOTOR4315_H

#include "ProjectConfig.h"


#ifdef MOTOR_COMPONENTS

#include "DeviceBase.h"
#include "Bus/RS485_Base.h"
#include "Motors/MotorBase.h"

template<int busID>
class Motor4315 : public MotorBase {
public:
    template<typename T>
    Motor4315(const Motor_Param_t&& params, T &_controller, uint32_t addr) : MotorBase(std::forward<const Motor_Param_t>(params)), rs485Agent(addr) {
        ResetController(_controller);
        this->SetDivisionFactor(20);
    }

    void SetTargetAngle(float targetAngle) {
        if(params.targetType != Motor_Ctrl_Type_e::Position) {
            return;
        }
        lastTarget = target;
        target = targetAngle + angleOffset;
        if(isInit)
        {
            lastTarget = target;
            isInit = false;
        }
        if(target-lastTarget<-180)
        {
            angleOffset += 360.f;
            target += 360.f;
        }
        if(target-lastTarget>180)
        {
            angleOffset -= 360.f;
            target -= 360.f;
        }
    }

    void Handle() override {
        Update();
        controller->Calc();
        MessageGenerate();
    }

    RS485_Agent<busID> rs485Agent;

private:

    float lastTarget = 0;
    float angleOffset = 0;
    bool isInit = true;

    void SetFeedback() override {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedback({&state.position});
                break;
        }
    }

    void MessageGenerate() {
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Position: {
                int32_t txAngle = controller->GetOutput() * 16384.0f / 360.0f;

                rs485Agent.txbuf[0] = 0x3E;//协议头
                rs485Agent.txbuf[1] = 0x00;//包序号
                rs485Agent.txbuf[2] = rs485Agent.addr; //ID
                rs485Agent.txbuf[3] = 0x55;//相对位置闭环控制命令码
                rs485Agent.txbuf[4] = 0x04;//数据包长度
                rs485Agent.txbuf[5] = txAngle;
                rs485Agent.txbuf[6] = txAngle >> 8u;
                rs485Agent.txbuf[7] = txAngle >> 16u;
                rs485Agent.txbuf[8] = txAngle >> 24u;
                uint16_t crc16 = CRC16Calc(rs485Agent.txbuf, 9);
                rs485Agent.txbuf[9] = crc16;
                rs485Agent.txbuf[10] = crc16 >> 8u;

                rs485Agent.rs485Write(rs485Agent.txbuf, 11);
                break;
            }
        }
    }

    void Update() {
        // state.position = (float) ((rs485Agent.rxbuf[7] | (rs485Agent.rxbuf[8] << 8u)) * 360.0f / 16384.0f);//单圈编码值
        // state.speed = (int16_t)(rs485Agent.rxbuf[11] | (rs485Agent.rxbuf[12] << 8u));
        // state.torque = 0;//电机应答不返回电流值
        // state.temperature = 0;//电机应答不返回温度参数
        state.position = target;
    }
/*    void AngleCalc() {
        state.position = (float) (rs485Agent.rxbuf[7] | (rs485Agent.rxbuf[8] << 8u) | (rs485Agent.rxbuf[9] << 16u) |
                                  (rs485Agent.rxbuf[10] << 24u)) / 16384.0f * 360.0f;
        if (targetAngle - lastAngle > 180) {
            zeroAngle -= 360;
        }
        if (lastAngle - targetAngle > 180) {
            zeroAngle += 360;
        }
        lastAngle = targetAngle;
        realAngle = targetAngle + zeroAngle;
        if (stopflag) {
            txAngle = (int32_t) (zeroAngle * 16384.0f / 360.0f);
        } else {
            txAngle = (int32_t) (realAngle * 16384.0f / 360.0f);
        }
    }*/

};

#endif

#endif //FINEMOTE_MOTOR4315_H
