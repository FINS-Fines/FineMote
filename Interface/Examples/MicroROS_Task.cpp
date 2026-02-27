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

#include "BSP_MicroROS.hpp"
#include "Bus/MicroROS_Base.hpp"

#include "Motors/Motor4010.hpp"
#include "Control/PID.hpp"

ROSAgent* ROSAgent::head_ = nullptr;

DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Int32, std_msgs, msg, Int32)
DEFINE_MICROROS_MSG_TYPE(std_msgs__msg__Bool, std_msgs, msg, Bool)

#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
constexpr PID_Param_t speedPID = {0.23f, 0.008f, 0.3f};
auto wheelControllers = CreateControllers<PID, 4>(speedPID);

Motor4010<1, 5> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);


void HeartbeatConverter(std_msgs__msg__Int32& msg, const int32_t& state) {
  msg.data = state;
}

RosPublisher<std_msgs__msg__Int32, int32_t> pub_heartbeat("system/heartbeat", HeartbeatConverter);

void OnLedCommand(const std_msgs__msg__Bool& msg) {
#ifdef LED_Pin
  HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
#endif
}

RosSubscriber<std_msgs__msg__Bool> sub_led("led_cmd", OnLedCommand);

void OnTimerCallback() {
  static int32_t counter = 0;
  pub_heartbeat.Update(counter++);
}

Timer timer_500ms(500, OnTimerCallback);

extern "C" void StartMicroROSTask(void* argument) {
  auto& RosManager = MicroROS_Base<5>::GetInstance();
  RosManager.Init();

  for (;;) {
    RosManager.Handle();

    osDelay(100);
  }
}