/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
******************************************************************************/

#include <cstring>

#include "ProjectConfig.h"
#include "cmsis_os.h"

#include "MicroROS/MicroROS_Agent.hpp"
#include "MicroROS/MicroROS_Manager.hpp"
#include "MicroROS/MicroROS_MessageTypes.hpp"

int32_t count_2 = 0;
RosPublisher pub_hb_2("heartbeat_2", [](std_msgs__msg__Int32& msg)
{
    msg.data = count_2++;
});

RosSubscriber sub_led("cmd/led", [](const std_msgs__msg__Bool& msg)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
});

