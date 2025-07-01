/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Task.h"
#include "PWM_Base.hpp"

void UserTask1() {
 PWM_Base<1>::GetInstance().SetDutyCycle(0.2f);
 PWM_Base<2>::GetInstance().SetDutyCycle(0.4f);
 PWM_Base<3>::GetInstance().SetDutyCycle(0.6f);
 PWM_Base<4>::GetInstance().SetDutyCycle(0.8f);
}
TASK_EXPORT(UserTask1);
