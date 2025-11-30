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
     float getPosition() const {
        if(!getPos) {
            return NULL; // 如果没有成功读取位置
        }
        return position;
    }

protected:
    float position{0};
    bool getPos = false; // 是否成功读取编码器位置
};
#endif // ENCODERBASE_HPP
