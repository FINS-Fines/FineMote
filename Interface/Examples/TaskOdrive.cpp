/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Scheduler.h"
#include "Odrive.hpp"
#include "PID.hpp"

#define Speed2POSITION {Motor_Ctrl_Type_e::Speed, Motor_Ctrl_Type_e::Position}

void TaskOdriveControl() {

    constexpr PID_Param_t pid_params = {
            .kp = 0.001,
            .ki = 0,
            .kd = 0,
            .iMax = 200,
            .outMax = 1.0f,
    };

    static PID pid1(pid_params);

    static Odrive<2> odrive_motor(Speed2POSITION, pid1, 0x02);

    // 设置目标位置
    static float target_position= 180; // 示例目标位置
    odrive_motor.SetTargetAngle(target_position);
}

TASK_EXPORT(TaskOdriveControl);