/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Task.h"
#include "PWM_Base.hpp"
#include "DSHOT.hpp"

void UserTask1() {
 // PWM_Base<1>::GetInstance().SetDutyCycle(0.2f);
 // PWM_Base<2>::GetInstance().SetDutyCycle(0.4f);
 // PWM_Base<3>::GetInstance().SetDutyCycle(0.6f);
 // PWM_Base<4>::GetInstance().SetDutyCycle(0.8f);
 DSHOT_PIN<1>::GetInstance().SetTargetThrottle(0);
 DSHOT_PIN<2>::GetInstance().SetTargetThrottle(0.1);
 DSHOT_PIN<3>::GetInstance().SetTargetThrottle(0.2);
 DSHOT_PIN<4>::GetInstance().SetTargetThrottle(0.3);
}
TASK_EXPORT(UserTask1);
