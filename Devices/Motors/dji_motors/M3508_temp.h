/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

//
// Created by wzj on 2024/9/6.
//

#ifndef FINEMOTE_M3508_TEMP_H
#define FINEMOTE_M3508_TEMP_H

#include "ProjectConfig.h"

#ifdef MOTOR_COMPONENTS

#include "DeviceBase.h"
#include "Bus/CAN_Base.h"
#include "Motors/MotorBase.h"
#include "Motors/dji_motors/dji_group_agent.h"

typedef struct dji_raw_feedback {
    bool initialized;
    uint16_t last_ecd;
    uint16_t ecd;
    uint16_t speed; // rpm
    uint16_t rx_torque;
    uint8_t temperature;
} dji_raw_feedback_t;

template<int busID>
class M3508 : public MotorBase {
public:
    template<typename T>
    M3508(const Motor_Param_t &&params, T &_controller, uint32_t addr) : MotorBase(
            std::forward<const Motor_Param_t>(params)), canAgent(addr) {
      ResetController(_controller);
    }

    void Handle() final {
      Update();
      controller->Calc();
      MessageGenerate();
    };

    dji_group_agent<busID> *groupAgent;
    CAN_Agent<busID> canAgent;

private:
    dji_raw_feedback_t raw{false};

    void SetFeedback() final {
      switch (params.targetType) {
        case Motor_Ctrl_Type_e::Position:
          controller->SetFeedback({&state.position, &state.speed});
          break;
        case Motor_Ctrl_Type_e::Speed:
          controller->SetFeedback({&state.speed});
          break;
      }
    }

    void MessageGenerate() {
      volatile int16_t txTorque = controller->GetOutput();
      INRANGE(txTorque, -16384, 16384);
      groupAgent->SetOutput(canAgent.addr, txTorque);
    }

    void Update() {
      if (!raw.initialized) {
        raw.ecd = (uint16_t) (canAgent.rxbuf[0] << 8 | canAgent.rxbuf[1]);
        raw.last_ecd = raw.ecd;
      } else {
        raw.last_ecd = raw.ecd;
        raw.ecd = (uint16_t) (canAgent.rxbuf[0] << 8 | canAgent.rxbuf[1]);
      }
      raw.speed = (uint16_t) (canAgent.rxbuf[2] << 8 | canAgent.rxbuf[3]);
      raw.rx_torque = (uint16_t) (canAgent.rxbuf[4] << 8 | canAgent.rxbuf[5]);
      raw.temperature = canAgent.rxbuf[6];

      if (abs(raw.ecd - raw.last_ecd) < 4096) {
        state.position += (raw.ecd - raw.last_ecd) * 360.f / 8192.f / params.reductionRatio;
      } else if (raw.ecd > raw.last_ecd) {
        state.position -= (8192 - (raw.ecd - raw.last_ecd)) * 360.f / 8192.f / params.reductionRatio;
      } else {
        state.position += (8192 - (raw.last_ecd - raw.ecd)) * 360.f / 8192.f / params.reductionRatio;
      }
      state.speed = raw.speed * 360.f / 60.f / params.reductionRatio; //dps
      state.torque = raw.rx_torque;
      state.temperature = raw.temperature;
    }
};

#endif //MOTOR_COMPONENTS
#endif //FINEMOTE_M3508_TEMP_H
