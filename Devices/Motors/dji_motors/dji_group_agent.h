/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

//
// Created by wzj on 2024/9/6.
//

#ifndef FINEMOTE_DJI_GROUP_AGENT_H
#define FINEMOTE_DJI_GROUP_AGENT_H

#include "ProjectConfig.h"

#ifdef MOTOR_COMPONENTS

#include "DeviceBase.h"
#include "Bus/CAN_Base.h"
#include "Motors/MotorBase.h"

typedef enum group_id {
    group_1to4,
    group_5to8,
    group_9to11,
    group_none,
} group_id_e;

template<int busID>
class dji_group_agent : public DeviceBase {
public:
    template<typename T>
    explicit dji_group_agent(uint32_t addr) :canAgent(addr) {
      switch (addr) {
        case (0x200):
          group = group_1to4;
          break;
        case (0x1FF):
          group = group_5to8;
          break;
        case (0x2FF):
          group = group_9to11;
          break;
        default:
          break;
      }
    }

    void Handle() final {
      canAgent.Send(canAgent.addr);
    };

    uint8_t SetOutput(uint32_t addr, int16_t cmd_Torque) {
      uint32_t idx = addr - 0x201;
      if (idx > 10) return 1;

      if (idx < 4) {
        if (group != group_1to4)return 1;
        canAgent[idx * 2] = cmd_Torque >> 8;
        canAgent[idx * 2 + 1] = cmd_Torque;
      } else if (idx < 8) {
        if (group != group_5to8) return 1;
        canAgent[(idx - 4) * 2] = cmd_Torque >> 8;
        canAgent[(idx - 4) * 2 + 1] = cmd_Torque;
      } else {
        if (group != group_9to11)return 1;
        canAgent[(idx - 8) * 2] = cmd_Torque >> 8;
        canAgent[(idx - 8) * 2 + 1] = cmd_Torque;
      }
      return 0;
    }

    group_id_e group = group_none;
    CAN_Agent<busID> canAgent;

private:
//    void Update() {
//
//    }
};

#endif //MOTOR_COMPONENTS
#endif //FINEMOTE_DJI_GROUP_AGENT_H
