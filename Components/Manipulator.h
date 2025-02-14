/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MANIPULATOR_H
#define FINEMOTE_MANIPULATOR_H

#include <cmath>

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "MotorBase.h"
#include "EncoderBase.h"

class Manipulator : public DeviceBase{
public:
    bool isInitFinished = false;
    bool GetInitCommand = false;

    Manipulator(MotorBase* _motorA, MotorBase* _motorB, MotorBase* _motorC, MotorBase* _motorD, MotorBase* _motorE,
                MotorBase* _motorF,MotorBase* _motorG,EncoderBase* _encoderC,EncoderBase* _encoderD,EncoderBase* _encoderE):
        motorA(_motorA), motorB(_motorB), motorC(_motorC), motorD(_motorD), motorE(_motorE), motorF(_motorF),motorG(_motorG),EncoderC(_encoderC),EncoderD(_encoderD),EncoderE(_encoderE){}

    void SetAngle(float AAngle, float BAngle, float CAngle, float DAngle, float EAngle, float FAngle)
    {
        targetAngle[0] = AAngle;
        targetAngle[1] = BAngle;
        targetAngle[2] = -CAngle + BAngle / reductionRatio[2]; //三关节与二关节耦合
        targetAngle[3] = DAngle;
        targetAngle[4] = -EAngle;
        targetAngle[5] = FAngle;
    }

    void SetEndEffectorAngle(const bool isOpen)
    {
        endEffectorState = isOpen;
    }

    void Handle() override
    {
        if(!isInitFinished)
        {
            UpdateEncoderData();
            ManipulatorInit();
        }
        SendAngle();
    }


private:
    bool endEffectorState = true;
    const float endEffectorCloseAngle = 298;
    const float endEffectorOpenAngle = 359;

    MotorBase* motorA;
    MotorBase* motorB;
    MotorBase* motorC;
    MotorBase* motorD;
    MotorBase* motorE;
    MotorBase* motorF;
    MotorBase* motorG;
    EncoderBase* EncoderC;
    EncoderBase* EncoderD;
    EncoderBase* EncoderE;

    float initialCAngle{0};
    float initialDAngle{0};
    float initialEAngle{0};

    float EncoderCAngle{0};
    float EncoderDAngle{0};
    float EncoderEAngle{0};

    const float CTargetAngle{220.4};
    const float DTargetAngle{48.4};
    const float ETargetAngle{71};
    const float CZeroPointAngle{214.5};//12
    const float DZeroPointAngle{44.7};//7.2
    const float EZeroPointAngle{47.9};//7.2
    const float CZP2Target = CTargetAngle - CZeroPointAngle;
    const float DZP2Target = DTargetAngle - DZeroPointAngle;
    const float EZP2Target = ETargetAngle - EZeroPointAngle;
    float targetAngle[6]{};
    float reductionRatio[6]{25, 30, 30, 50, 50, 1};
    float angleOffset[7]{0, 0, 0, 0, 0, 0};

    enum State
    {
        WaitForOdriveInit,
        GetEncoderData,
        WaitForInitCommand,
        InitThirdJoint,
        InitOtherJoint,
        Finish
    };
    State state = WaitForOdriveInit;

    void ManipulatorInit()
    {
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
                if(EncoderCAngle == 0 || EncoderDAngle == 0)// || EncoderEAngle == 0)
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
                angleOffset[2] = CZP2Target + 12.0f *std::roundf((CZeroPointAngle - initialCAngle)/12.0f);
                // angleOffset[2]=CTargetAngle-initialCAngle;
                motorC->Enable();

                counter++;
                if(counter>5000)
                {
                    state = InitOtherJoint;
                    counter = 0;
                }
                break;
            case InitOtherJoint:
                angleOffset[0] = -4.6;
                angleOffset[1] = 80.3;
                // angleOffset[3] = -(DTargetAngle-initialDAngle);
                // angleOffset[4] = -(ETargetAngle-initialEAngle);
                angleOffset[3] = -(DZP2Target + 7.2f *std::roundf((DZeroPointAngle - initialDAngle)/7.2f));
                angleOffset[4] = -(EZP2Target + 7.2f *std::roundf((EZeroPointAngle - initialEAngle)/7.2f));
                angleOffset[5] = 195;
                angleOffset[6] = endEffectorCloseAngle;

                motorA->Enable();
                motorB->Enable();
                motorD->Enable();
                motorE->Enable();
                motorF->Enable();
                motorG->Enable();

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
        motorG->SetTargetAngle(endEffectorState?endEffectorOpenAngle:endEffectorCloseAngle);
    }

    void UpdateEncoderData()
    {
        EncoderCAngle = EncoderC->GetAngle();
        EncoderDAngle = EncoderD->GetAngle();
        EncoderEAngle = EncoderE->GetAngle();
    }
};

#endif //FINEMOTE_MANIPULATOR_H
