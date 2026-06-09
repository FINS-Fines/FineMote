/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "DeviceBase/Task.hpp"

[[maybe_unused]] static auto& user_task1 = make_task(
    [] {
        // Do something
    },
    100);
