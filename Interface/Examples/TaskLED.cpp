// Copyright (c) 2025.
// IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
// All rights reserved.

#include "DeviceBase/Task.hpp"

#include "MultiMedia/LED.h"

[[maybe_unused]] static auto& task_led = make_task(
    [] {
        static uint16_t cnt = 0;
        cnt++;
        if (cnt > 1000) {
            cnt = 0;
            LED::Toggle();
        }
    },
    1
);
