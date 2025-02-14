//
// Created by gan_ju on 25-2-14.
//

#ifndef H30_IMU_H
#define H30_IMU_H

#include "Chassis.h"
#include "ProjectConfig.h"
#include "DeviceBase.h"

class H30_imu {
private:
    typedef struct{
        uint8_t dataId = 0x10;
        uint8_t dataLen = 12;
        int32_t x{0};
        int32_t y{0};
        int32_t z{0};
    }Acc;//加速度
    Acc acc_;

    typedef struct{
        uint8_t dataId = 0x20;
        uint8_t dataLen = 12;
        int32_t x{0};
        int32_t y{0};
        int32_t z{0};
    }AngVel;//角速度
    AngVel ang_vel_;

    typedef struct{
        uint8_t dataId = 0x40;
        uint8_t dataLen = 12;
        float pitch{0};
        float roll{0};
        float yaw{0};
    }Euler;//欧拉角
    Euler euler_;

    typedef struct{
        uint8_t dataId = 0x41;
        uint8_t dataLen = 16;
        int32_t q0{0};
        int32_t q1{0};
        int32_t q2{0};
        int32_t q3{0};
    }Quat;//四元数
    Quat quat_;

    uint8_t rxBUffer[80]{};

public:
    float GetYaw()
    {
        return euler_.yaw;
    }

    void Decode(uint8_t* data, uint16_t length)
    {
        if (data[0] == 0x59 && data[1] == 0x53 && data[4] == 0x3c && H30CheckSum(data + 2, length - 4) == data[length -
            2] << 8 | data[length - 1])
        {
            memcpy(rxBUffer, data, length);
            if (data[33] == euler_.dataId)
            {
                euler_.pitch = (float)(data[35] | data[36] << 8 | data[37] << 16 | data[38] << 24) * 0.000001 / 180.f * PI;
                euler_.roll = (float)(data[39] | data[40] << 8 | data[41] << 16 | data[42] << 24) * 0.000001 / 180.f * PI;
                float yaw = (float)(data[43] | data[44] << 8 | data[45] << 16 | data[46] << 24) * 0.000001 / 180.f * PI;
                while (euler_.yaw - yaw > 1.9 * PI)
                {
                    yaw += 2*PI;
                }
                while (euler_.yaw - yaw < -1.9 * PI)
                {
                    yaw -= 2*PI;
                }
                euler_.yaw = yaw;
            }
        }
    }

    uint16_t H30CheckSum(uint8_t* data,uint16_t len)
    {
        uint8_t ck1=0,ck2=0;
        for(uint16_t i=0;i<len;i++)
        {
            ck1 += data[i];
            ck2 += ck1;
        }
        uint16_t ck = (ck1<<8)|ck2;
        return ck;
    }
};




#endif //H30_IMU_H
