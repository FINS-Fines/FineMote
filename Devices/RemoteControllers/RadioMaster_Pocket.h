/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_RADIOMASTER_POCKET_H
#define FINEMOTE_RADIOMASTER_POCKET_H

#include "ProjectConfig.h"
#include "RemoteControl.h"

class RadioMaster_Pocket : public RemoteControl {
public:
    RadioMaster_Pocket() {}
    void Decode(uint8_t *data, uint16_t length) override;

private:
    static constexpr uint32_t SBUS_RX_BUF_NUM = 50;
    static constexpr int32_t RC_CH_VALUE_OFFSET = 1024;
    int32_t channel[8] = {};
};


#endif
