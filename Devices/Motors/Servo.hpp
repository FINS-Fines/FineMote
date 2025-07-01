/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_SERVO_H
#define FINEMOTE_SERVO_H

#include "Bus/PWM_Base.hpp"

/**
 * 只考虑了180角度舵机
 */
template<uint32_t deviceID>
class Servo {
private:
    float period = 20; //ms
    float startDuration = 0.5;//ms
    float endDuration = 2.5;//ms
    float fullAngle = 180; //角度
public:
    void SetAngle(float angle){
        float compare = angle / fullAngle * (endDuration - startDuration) + startDuration;
        compare = compare/period;
        PWM_Base<deviceID>::GetInstance().SetDutyCycle(compare);
    };
};

#endif
