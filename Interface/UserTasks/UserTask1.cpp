/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "Task.h"

#include "Bus/UART_Base.hpp"

uint8_t buffer[] = "Hello";

void UserTask1() {
 // Do something
 static uint32_t cnt = 0;
 cnt++;
 if (cnt > 1000) {
  UART_Base<6>::GetInstance().Transmit(buffer, sizeof(buffer));
  cnt = 0;
 }
}
TASK_EXPORT(UserTask1);