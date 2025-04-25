/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

//
// Created by wzj on 2024/9/6.
//

#include <cmath>
#include "Chassis_differential.h"


void Chassis_d::ChassisSetVelocity(float _fbV, float _rtV) {
  ChassisStopFlag = false;
  if (FBVelocity >= 0) {
    if (_fbV > FBVelocity + acc_limit) FBVelocity = FBVelocity + acc_limit;
    else if (_fbV < FBVelocity - deacc_limit) FBVelocity = FBVelocity - deacc_limit;
    else FBVelocity = _fbV;
  } else {
    if (_fbV < FBVelocity - acc_limit) FBVelocity = FBVelocity - acc_limit;
    else if (_fbV > FBVelocity + deacc_limit) FBVelocity = FBVelocity + deacc_limit;
    else FBVelocity = _fbV;
  }

  if (!chassisAutoFlag) {
    RTVelocity = _rtV;
    return;
  }
  if (RTVelocity >= 0) {
    if (_rtV > RTVelocity + ang_acc_limit) RTVelocity = RTVelocity + ang_acc_limit;
    else if (_rtV < RTVelocity - ang_deacc_limit) RTVelocity = RTVelocity - ang_deacc_limit;
    else RTVelocity = _rtV;
  } else {
    if (_rtV < RTVelocity - ang_acc_limit) RTVelocity = RTVelocity - ang_acc_limit;
    else if (_rtV > RTVelocity + ang_deacc_limit) RTVelocity = RTVelocity + ang_deacc_limit;
    else RTVelocity = _rtV;
  }

//  RTVelocity = _rtV;
}

void Chassis_d::ChassisStop() {
  ChassisStopFlag = true;

  LMotor.Stop();
  LMotor.Stop();
}

void Chassis_d::WheelsSpeedCalc(float fbVelocity, float rtVelocity) {
  float ChassisSpeed[2];

  float v = fbVelocity; // m/s
  float w = rtVelocity / 180.f * PI; // rad/s

  /// TODO speed limit
  //计算四个轮子线速度，单位：度/s
  ChassisSpeed[0] = (v - (WIDTH * w) / 2) / (WHEEL_DIAMETER * PI) * 360;//左轮
  ChassisSpeed[1] = (v + (WIDTH * w) / 2) / (WHEEL_DIAMETER * PI) * 360;//右轮


  //设置底盘电机转速
  LMotor.SetTargetSpeed(ChassisSpeed[0]);
  RMotor.SetTargetSpeed(ChassisSpeed[1]);

}

void Chassis_d::WheelOdometer() {
  OdoFeedback.FBV = (LMotor.GetState().speed + RMotor.GetState().speed) / 2 * (PI / 360) * (WHEEL_DIAMETER);
  OdoFeedback.RTV = (RMotor.GetState().speed - LMotor.GetState().speed) / WIDTH * (PI / 360) * (WHEEL_DIAMETER);
  if (abs(OdoFeedback.RTV) > 0.25f) OdoFeedback.RTV *= 0.8f;

  OdoFeedback.x = OdoFeedback.x + 0.001f * OdoFeedback.FBV * cosf(OdoFeedback.theta);
  OdoFeedback.y = OdoFeedback.y + 0.001f * OdoFeedback.FBV * sinf(OdoFeedback.theta);
  OdoFeedback.theta = OdoFeedback.theta + 0.001f * OdoFeedback.RTV;

  if (OdoFeedback.theta > 180.f) OdoFeedback.theta -= 360.f;
  else if (OdoFeedback.theta <= -180.f) OdoFeedback.theta += 360.f;
}

void Chassis_d::Handle() {
  if (chassisAutoFlag) AutoTracker();
  WheelsSpeedCalc(FBVelocity, RTVelocity);
  WheelOdometer();
}

Chassis_d_Builder Chassis_d::Build() {
  return {};
}

void Chassis_d::AutoTracker() {
  if (QueueHead == QueueTail) {
    ChassisSetVelocity(0, 0);
    return;
  };


  expTheta = atan2f(pointQueue[next].y - OdoFeedback.y, pointQueue[next].x - OdoFeedback.x);
  angError = (expTheta - OdoFeedback.theta) * 180 / PI;
  if (angError > 180.f)angError -= 360.f;
  else if (angError < -180.f)angError += 360.f;
  disError = sqrt(powf(pointQueue[next].y - OdoFeedback.y, 2) + powf(pointQueue[next].x - OdoFeedback.x, 2));

  switch (autoState) {
    case UPDATE:
      curr = QueueHead;
      next = (QueueHead + 1) % 10;
      stateCnt = 6000;
      autoState = DIRECTION;
      break;
    case DIRECTION:
      if (fabs(angError) > 5.f && stateCnt-- > 0) {
        ChassisSetVelocity(0, angError * 4.f);
      } else {
        ChassisSetVelocity(0, 0);
        stateCnt = 5000 + int(1000.f * disError);
        autoState = STRAIGHT;
      }
      break;
    case STRAIGHT:
      expSpeed = disError * 3.f > TrackingSpeed ? TrackingSpeed : disError * 3.f;
      ChassisSetVelocity(expSpeed, angError * 3.f);
      if (disError < 0.2 || stateCnt-- <= 1) {
        ChassisSetVelocity(0, 0);
        QueueHead = next;
        autoState = UPDATE;
      }
      break;
  }


}


