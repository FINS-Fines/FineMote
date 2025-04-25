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
#define acc_limit 0.001f //*1000 m/(s^2)
#define deacc_limit 0.006f //*1000 m/(s^2)
#define TrackingSpeed 1.0f //m/s
#define ang_acc_limit 0.15f //*1000 rad/(s^2)
#define ang_deacc_limit 1.f //*1000 rad/(s^2)

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

    void WheelOdometer();

    void AutoTracker();

    void Handle() override;

    bool ChassisStopFlag = true;
    float FBVelocity{}, RTVelocity{};
    struct OdoFeedback_t {
        float FBV{}, RTV{}; // m/s, rad/s
        float lastLAngle{}, lastRAngle{};
        float x{}, y{}, theta{}; // m, m, rad
    } OdoFeedback;

    MotorBase &LMotor, &RMotor;

    uint8_t chassisAutoFlag = false;

    enum AutoState_e {
        UPDATE,
        DIRECTION,
        STRAIGHT,
    } autoState = UPDATE;

    struct TrackPoint_t {
        float x{}, y{}, theta{};
    };

    uint8_t QueueHead = 0; //当前所在点
    uint8_t QueueTail = 4; //最后一个有效点

    uint8_t curr, next;
    float expTheta, expSpeed;
    uint32_t stateCnt;
    float angError, disError;

    TrackPoint_t pointQueue[10] = {
            {0.0f, 0.0f,  0.f},  // 第一个点（固定为000）
            {2.5f, 0.0f,  0.f}, // 第二个点
            {2.5f, 2.5f,  0.f},  // 第三个点
            {0.0f, 2.5f,  0.f},
            {0.0f, 0.0f,  0.f},
            {6.0f, 7.0f,  0.f},
            {7.0f, 8.0f,  0.f},
            {8.0f, 9.0f,  0.f},
            {9.0f, 10.0f, 0.f},
            {9.0f, 9.0f,  0.f} // 第十个点
    };

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
