/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_DJI_DT7_H
#define FINEMOTE_DJI_DT7_H

#include "ProjectConfig.h"

#ifdef DJI_DT7_MODULE

#include "RemoteControl.h"
#include "Bus/UART_Base.h"

class DT7 : public RemoteControl{
public:
    void Decode(uint8_t*, uint16_t) override {};

private:
    static constexpr uint32_t SBUS_RX_BUF_NUM = 50;
    static constexpr int32_t RC_CH_VALUE_OFFSET = 1024;
    uint8_t rxBuff[SBUS_RX_BUF_NUM];
    int32_t channel[16];
    UART_Agent<0> remoteUart;
};

#endif //DJI_DT7_MODULE

#endif //FINEMOTE_DJI_DT7_H
