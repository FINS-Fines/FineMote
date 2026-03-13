/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#include <cstring>

#include "ProjectConfig.h"
#include "cmsis_os.h"

#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>
#include <sensor_msgs/msg/joint_state.h>

#include "BSP_MicroROS.hpp"
#include "Bus/MicroROS_Base.hpp"

#include "Motors/Motor4010.hpp"
#include "Control/PID.hpp"

DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Int32, std_msgs, msg, Int32)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Bool, std_msgs, msg, Bool)
DEFINE_MICROROS_MSG_TYPE(sensor_msgs__msg__JointState, sensor_msgs, msg, JointState)

#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
constexpr PID_Param_t speedPID = {0.23f, 0.008f, 0.3f};
auto wheelControllers = CreateControllers<PID, 4>(speedPID);

Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);

RosPublisher pub_motor_state(
    "motor/cbr/state",
    CBRMotor
);

int32_t count = 0;
RosPublisher pub_hb("heartbeat", [](std_msgs__msg__Int32& msg) {
    msg.data = count++;
});

RosSubscriber sub_led("cmd/led", [](const std_msgs__msg__Bool& msg) {
        HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
});

void OnTimerCallback() {
    // heartbeat++ ;
}

Timer<WITH_MICRO_ROS> timer_500ms(500, OnTimerCallback);

extern "C" void StartMicroROSTask(void* argument) {
  auto& RosManager = MicroROS_Base<>::GetInstance();
  // RosManager.Init();
  for (;;) {
    RosManager.Handle();

    osDelay(100);
  }
}