/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "FreeRTOS.h"
#include "task.h"
#include "cmsis_os.h" // FIX: 必须包含，否则 osDelay 报错

#include "MicroROS_Manager.hpp"
#include "PingPongApp.hpp"
#include "MicroROSPort.hpp"

// 定义使用的端口类型 (例如 UART5)
using MyMicroROSPort = MicroROSPort<5>;

// 实例化应用
PingPongApp pingPongApp;

extern "C" void StartMicroROSTask(void *argument) {

    // 1. 获取管理器实例
    auto &manager = MicroROSManager<MyMicroROSPort>::GetInstance();

    // 2. 注册业务逻辑
    manager.RegisterApp(&pingPongApp);

    // 3. 任务循环
    while (true) {
        manager.RunLoop();
        osDelay(10); // 让出CPU
    }
}