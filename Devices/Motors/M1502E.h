/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_M1502E_H
#define FINEMOTE_M1502E_H

#include "ProjectConfig.h"

#ifdef MOTOR_COMPONENTS

#include "Bus/CAN_Base.h"
#include "Motors/MotorBase.h"


template<int busID>
class M1502E : public MotorBase {
public:

    template<typename T>
    M1502E(const Motor_Param_t&& params, T& _controller, uint32_t addr) : MotorBase(std::forward<const Motor_Param_t>(params)), canAgent(addr) {
        SetDivisionFactor(1);//设置分频系数
        ResetController(_controller);
        SetFeedback_1kHz();
//        ChooseCtrlType();
        ///Todo:在BUG：电机初始化的时候无法发送多条CAN消息，但是在断点调试的时候可以，所以查不出BUG，换个地方发送规避问题

    }//构造函数

    void Handle() final{
        Update();
        controller->Calc();
        MessageGenerate();
    };

    CAN_Agent<busID> canAgent;

private:
    bool mode_updated = false;
    void SetFeedback() final{
        switch (params.targetType) {
            case Motor_Ctrl_Type_e::Position:
                controller->SetFeedback({&state.position, &state.speed});
                break;
            case Motor_Ctrl_Type_e::Speed:
                controller->SetFeedback({&state.speed});
                break;
        }
    }
    void SetFeedback_1kHz(){
        canAgent[0] = 0x01;
        canAgent[1] = 0x01;
        canAgent[2] = 0x01;
        canAgent[3] = 0x01;
        canAgent[4] = 0x01;
        canAgent[5] = 0X01;
        canAgent[6] = 0x01;
        canAgent[7] = 0x01;
        canAgent.Send(0x106);
    }
    void ChooseCtrlType(){//改变电机的控制方式：电流环、速度环、位置环, 这东西在开机的时候（在构造函数里）自动执行一次
        switch (params.ctrlType) {
            //力矩环下，力矩（电流）代表电机在该力矩下运行
            case Motor_Ctrl_Type_e::Torque: {
                canAgent[0] = 0x01;
                canAgent[1] = 0x01;
                canAgent[2] = 0x01;
                canAgent[3] = 0x01;
                canAgent[4] = 0x01;
                canAgent[5] = 0X01;
                canAgent[6] = 0x01;
                canAgent[7] = 0x01;
                break;
            }
                //位置速度力矩三闭环模式下,速度命令代表电机在位置控制下可达到的最大速度,力矩命令代表电机可达到的最大力矩
            case Motor_Ctrl_Type_e::Position: {
                canAgent[0] = 0x03;
                canAgent[1] = 0x03;
                canAgent[2] = 0x03;
                canAgent[3] = 0x03;
                canAgent[4] = 0x03;
                canAgent[5] = 0X03;
                canAgent[6] = 0x03;
                canAgent[7] = 0x03;
                break;
            }
                //速度力矩环下，速度命令代表电机运行速度，力矩（电流）代表电机在该速度下运行，能提供的最大电流
            case Motor_Ctrl_Type_e::Speed: {
                canAgent[0] = 0x02;
                canAgent[1] = 0x02;
                canAgent[2] = 0x02;
                canAgent[3] = 0x02;
                canAgent[4] = 0x02;
                canAgent[5] = 0X02;
                canAgent[6] = 0x02;
                canAgent[7] = 0x02;
                break;
            }
        }
        canAgent.Send(0x105);
    }

    void MessageGenerate() {
        if (!mode_updated){
            ChooseCtrlType();
            mode_updated = true;
        }
        switch (params.ctrlType) {
            case Motor_Ctrl_Type_e::Torque: {
                int16_t txTorque = Clamp(-controller->GetOutput(), -500.f, 500.f);

                canAgent[0] = txTorque;
                canAgent[1] = txTorque >> 8;
                canAgent[2] = 0x00;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
            case Motor_Ctrl_Type_e::Speed: {
                float inputSpeed = controller->GetOutput(); // 获取输入的速度值
                int32_t mappedValue = static_cast<int32_t>(inputSpeed * (21000.0f / 210.0f)); // 映射到 -21000 到 +21000

                // 将映射值拆分为两个字节
                canAgent[0] = static_cast<uint8_t>((mappedValue >> 8) & 0xFF); // 高字节
                canAgent[1] = static_cast<uint8_t>(mappedValue & 0xFF);       // 低字节
                canAgent[2] = 0x00;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;

                break;
            }
            case Motor_Ctrl_Type_e::Position: {//zsy：不保熟
                constexpr uint16_t txSpeed = 0x800;
                float targetAngle = -controller->GetOutput();

                int32_t txAngle = 100 * targetAngle * -1;//统一正方向
                canAgent[0] = txAngle;
                canAgent[1] = txAngle >> 8;
                canAgent[2] = 0x00;
                canAgent[3] = 0x00;
                canAgent[4] = 0x00;
                canAgent[5] = 0x00;
                canAgent[6] = 0x00;
                canAgent[7] = 0x00;
                break;
            }
        }
        canAgent.Send(0x32);//标识符0x32，我只管ID为1的电机，其他一律不管（因为我消防机器人身上只有一个M1502E）
    }

    void Update() {
        state.position = (int16_t)(canAgent.rxbuf[4] << 8u | (canAgent.rxbuf[5])) * 360.f / 32767.f;
        state.speed = (int16_t)(canAgent.rxbuf[0] << 8u | (canAgent.rxbuf[1])) * 210.f / 21000.f;//单位：rpm
        state.torque = (int16_t)(canAgent.rxbuf[2] << 8u | (canAgent.rxbuf[3]))* 33.f / 32767.f;
        state.temperature = 0.0;
    }
};

#endif //MOTOR_COMPONENTS
#endif //FINEMOTE_M1502E_H
