/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

//
// Created by wzj on 2024/9/6.
//

#include "Chassis_differential.h"


void Chassis_d::ChassisSetVelocity(float _fbV, float _rtV) {
  ChassisStopFlag = false;
  FBVelocity = _fbV;
  RTVelocity = _rtV;
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
  ChassisSpeed[0] = (v + (WIDTH * w) / 2) / (WHEEL_DIAMETER * PI) * 360;//左轮
  ChassisSpeed[1] = (v - (WIDTH * w) / 2) / (WHEEL_DIAMETER * PI) * 360;//右轮


  //设置底盘电机转速
  LMotor.SetTargetSpeed(ChassisSpeed[0]);
  RMotor.SetTargetSpeed(ChassisSpeed[1]);

}

void Chassis_d::Handle() {
  WheelsSpeedCalc(FBVelocity, RTVelocity);
}

Chassis_d_Builder Chassis_d::Build() {
  return {};
}
