/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#include <cstring>

#include "ProjectConfig.h"
#include "cmsis_os.h"

#include <sensor_msgs/msg/joint_state.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

#include "MicroROS/MicroROS_Agent.hpp"
#include "MicroROS/MicroROS_Manager.hpp"

#include "Control/PID.hpp"
#include "Motors/Motor4010.hpp"

DEFINE_MICROROS_MSG(std_msgs__msg__Int32, std_msgs, msg, Int32)
DEFINE_MICROROS_MSG(std_msgs__msg__Bool, std_msgs, msg, Bool)
DEFINE_MICROROS_MSG(sensor_msgs__msg__JointState, sensor_msgs, msg, JointState)

#define TORQUE_2_SPEED { Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed }
constexpr PID_Param_t speedPID = { 0.23f, 0.008f, 0.3f };
auto wheelControllers = CreateControllers<PID, 4>(speedPID);

Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);
auto pub_motor_state = MAKE_PUBLISHER(CBRMotor);

int32_t count = 0;
RosPublisher pub_hb("heartbeat", [](std_msgs__msg__Int32& msg) {
    msg.data = count++;
});

RosSubscriber sub_led("cmd/led", [](const std_msgs__msg__Bool& msg) {
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
});

extern "C" void StartMicroROSTask(void* argument) {
    auto& RosManager = MicroROS_Manager<>::GetInstance();

    for (;;) {
        RosManager.Handle();

        osDelay(100);
    }
}