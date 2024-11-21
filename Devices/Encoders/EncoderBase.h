/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef ENCODERBASE_H
#define ENCODERBASE_H

#include "ProjectConfig.h"
#include "DeviceBase.h"


class EncoderBase : public DeviceBase
{
public:
    float GetAngle()
    {
        return angle;
    }
protected:
    float angle{0};
};


#endif //ENCODERBASE_H
