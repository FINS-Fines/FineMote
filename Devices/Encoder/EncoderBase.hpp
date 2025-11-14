/*******************************************************************************
* Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef ENCODERBASE_HPP
#define ENCODERBASE_HPP

#include "ProjectConfig.h"
#include "DeviceBase.h"

class EncoderBase : public DeviceBase {
public:

protected:
    float position{0};
    uint8_t data_byte4{0};
    bool getPos = false; // 是否成功读取编码器位置
    uint8_t crc_value{0};
};
#endif // ENCODERBASE_HPP
