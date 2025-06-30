/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Task.h"
#include "PWM.h"

void UserTask1() {
 ChannelPWM<8,1>::GetInstance().SetCompare(0.5);
 ChannelPWM<8,1>::GetInstance().SetFrequency(50);//
}
TASK_EXPORT(UserTask1);
