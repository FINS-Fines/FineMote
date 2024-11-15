/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MANIPULATOR_H
#define FINEMOTE_MANIPULATOR_H

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "MotorBase.h"

class Manipulator : public DeviceBase
{
private:
    MotorBase* motorA;
    MotorBase* motorB;
    MotorBase* motorC;
    MotorBase* motorD;
    MotorBase* motorE;
    MotorBase* motorF;
    // MotorBase* motorG;

    float initialCAngle{0};
    float initialDAngle{0};
    float initialEAngle{0};

    float EncoderCAngle{0};
    float EncoderDAngle{0};
    float EncoderEAngle{0};

    const float CTargetAngle{222};
    const float DTargetAngle{49};
    const float ETargetAngle{230};
    float targetAngle[6]{};
    float reductionRatio[6]{25, 30, 30, 50, 50, 1};
    float angleOffset[6]{0, 0, 0, 0, 0, 0};

public:
    bool isInitFinished = false;
    bool GetInitCommand = false;
    Manipulator(MotorBase* _motorA, MotorBase* _motorB, MotorBase* _motorC, MotorBase* _motorD, MotorBase* _motorE,
                MotorBase* _motorF):
        motorA(_motorA), motorB(_motorB), motorC(_motorC), motorD(_motorD), motorE(_motorE), motorF(_motorF)
    {
    }

    void UpdataEncoderData(float CAngle, float DAngle, float EAngle)
    {
        EncoderCAngle = CAngle;
        EncoderDAngle = DAngle;
        EncoderEAngle = EAngle;
    }

    void SetAngle(float AAngle, float BAngle, float CAngle, float DAngle, float EAngle, float FAngle)
    {
        targetAngle[0] = AAngle;
        targetAngle[1] = BAngle;
        targetAngle[2] = -CAngle;
        targetAngle[3] = DAngle;
        targetAngle[4] = -EAngle;
        targetAngle[5] = FAngle;
    }

    void ManipulatorInit()
    {
        enum State
        {
            WaitForOdriveInit,
            GetEncoderData,
            WaitForInitCommand,
            InitThirdJoint,
            InitOtherJoint,
            Finish
        };
        static State state = WaitForOdriveInit;


        static uint32_t counter{0};

        switch (state)
        {
            case WaitForOdriveInit:
                counter++;
                if(counter>6000)
                {
                    counter = 0;
                    state = GetEncoderData;
                }
                break;
            case GetEncoderData:
                if(EncoderCAngle == 0 || EncoderDAngle == 0 || EncoderEAngle == 0)
                {
                    break;
                }
                initialCAngle+=EncoderCAngle/2000;
                initialDAngle+=EncoderDAngle/2000;
                initialEAngle+=EncoderEAngle/2000;
                counter++;
                if(counter>=2000)
                {
                    state = WaitForInitCommand;
                    counter=0;
                }
                break;
            case WaitForInitCommand:
                if(GetInitCommand)
                {
                    state = InitThirdJoint;
                }
                break;
            case InitThirdJoint://先初始化三关节，防止打到五关节编码器
                angleOffset[2]=CTargetAngle-initialCAngle;
                counter++;
                if(counter>5000)
                {
                    state = InitOtherJoint;
                    counter = 0;
                }
                break;
            case InitOtherJoint:
                angleOffset[0] = 5;
                angleOffset[1] = 80;
                angleOffset[3] = -(DTargetAngle-initialDAngle);
                angleOffset[4] = -(ETargetAngle-initialEAngle);
                counter++;
                if(counter>5000)
                {
                    state=Finish;
                }
                break;
            case Finish:
                isInitFinished = true;
                break;
        }
    }

    void SendAngle()
    {
        motorA->SetTargetAngle((targetAngle[0] + angleOffset[0]) * reductionRatio[0]);
        motorB->SetTargetAngle((targetAngle[1] + angleOffset[1]) * reductionRatio[1]);
        motorC->SetTargetAngle((targetAngle[2] + angleOffset[2]) * reductionRatio[2]);
        motorD->SetTargetAngle((targetAngle[3] + angleOffset[3]) * reductionRatio[3]);
        motorE->SetTargetAngle((targetAngle[4] + angleOffset[4]) * reductionRatio[4]);
        motorF->SetTargetAngle((targetAngle[5] + angleOffset[5]) * reductionRatio[5]);
    }


    void Handle() override
    {
        ManipulatorInit();
        SendAngle();
    }
};

#endif //FINEMOTE_MANIPULATOR_H
