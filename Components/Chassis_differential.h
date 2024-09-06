/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

//
// Created by wzj on 2024/9/6.
//

#ifndef FINEMOTE_CHASSIS_DIFFERENTIAL_H
#define FINEMOTE_CHASSIS_DIFFERENTIAL_H

#include "ProjectConfig.h"

#include "DeviceBase.h"
#include "MotorBase.h"

#define WIDTH  0.352f // 轮中心矩离
#define WHEEL_DIAMETER 0.18f // 轮直径, m
#define PI 3.1415926f

class Chassis_d_Builder;

class Chassis_d : public DeviceBase {
    friend class Chassis_d_Builder;

    Chassis_d(
            MotorBase *LMotorPtr,
            MotorBase *RMotorPtr) :
            LMotor(*LMotorPtr),
            RMotor(*RMotorPtr) {}

public:
    void ChassisSetVelocity(float _fbV, float _rtV);

    void ChassisStop();

    void WheelsSpeedCalc(float fbVelocity, float rtVelocity);

    void Handle() override;

    bool ChassisStopFlag = true;
    float FBVelocity{}, RTVelocity{};

    MotorBase &LMotor, &RMotor;

    static Chassis_d_Builder Build();
};

//TODO 实际上建造者应该只使用电机基类指针，底盘类使用的应该也是电机基类指针
class Chassis_d_Builder {
private:
    MotorBase *LMotorPtr{nullptr};
    MotorBase *RMotorPtr{nullptr};

public:
    Chassis_d_Builder &AddLMotor(MotorBase &lMotor) {
      LMotorPtr = &lMotor;
      return *this;
    };

    Chassis_d_Builder &AddRMotor(MotorBase &rMotor) {
      RMotorPtr = &rMotor;
      return *this;
    };


    Chassis_d Build() {
      if (!(LMotorPtr && RMotorPtr)) {
        Error_Handler();
      }
      return Chassis_d{LMotorPtr, RMotorPtr};

    }
};

#endif //FINEMOTE_CHASSIS_DIFFERENTIAL_H
