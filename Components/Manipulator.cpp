/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Manipulator.h"

// void Manipulator::UpdataEncoderData(float CAngle,float DAngle,float EAngle)
// {
//     EncoderCAngle = CAngle;
//     EncoderDAngle = DAngle;
//     EncoderCAngle = EAngle;
// }
//
// void Manipulator::SetAngle(float AAngle,float BAngle,float CAngle,float DAngle,float EAngle,float FAngle)
// {
//     targetAngle[0] = AAngle;
//     targetAngle[1] = BAngle;
//     targetAngle[2] = CAngle;
//     targetAngle[3] = DAngle;
//     targetAngle[4] = EAngle;
//     targetAngle[5] = FAngle;
// }
//
// void Manipulator::ManipulatorInit()
// {
//     static bool isInitFinished = false;
//     static bool isThirdJointInit = false;
//     if(isInitFinished){return;}
//     //先初始化三关节，防止打到五关节编码器
//     if(!isThirdJointInit)
//     {
//         // targetAngle[2] =
//     }
//     //其次初始化其余关节
// }
//
// void Manipulator::SendAngle()
// {
//     motorA->SetTargetAngle((targetAngle[0]+angleOffset[0])*reductionRatio[0]);
//     motorB->SetTargetAngle((targetAngle[1]+angleOffset[1])*reductionRatio[1]);
//     motorC->SetTargetAngle((targetAngle[2]+angleOffset[2])*reductionRatio[2]);
//     motorD->SetTargetAngle((targetAngle[3]+angleOffset[3])*reductionRatio[3]);
//     motorE->SetTargetAngle((targetAngle[4]+angleOffset[4])*reductionRatio[4]);
//     motorF->SetTargetAngle((targetAngle[5]+angleOffset[5])*reductionRatio[5]);
// }
//
//
// void Manipulator::Handle() override {
//     SendAngle();
// }

