/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Robomaster_A.h"

#ifdef __cplusplus
extern "C" {
#endif

void BSP_Setup() {
    HAL_TIM_Base_Start_IT(&TIM_Control);
}

#ifdef __cplusplus
}
#endif
