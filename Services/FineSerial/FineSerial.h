/*******************************************************************************
* Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef FINESERIAL_H
#define FINESERIAL_H

#include "ProjectConfig.h"
#include "DeviceBase.h"
#include "UARTBaseLite.h"
#include "Verify.h"

template <uint8_t busID>
class FineSerial : public DeviceBase
{
public:
    enum class CommandType{
        NONE = 0x00,
        MOVE_CHASSIS = 0X01,
        MOVE_MANIPULATOR = 0X02,
        ODOMETRY_OFFSET = 0X03,
        CHASSIS_STOP = 0X04,
        SET_PATH_POINT = 0x05,
    } command_type = CommandType::NONE;

    static FineSerial& GetInstance(){
        static FineSerial instance;
        return instance;
    }

    //单例模式删除拷贝构造运算符
    FineSerial(const FineSerial&) = delete;
    FineSerial& operator=(const FineSerial&) = delete;

    void Decode(uint8_t* data, uint16_t size){
        uint8_t commandLength = data[2];
        if(data[0] == 0xAA && data[size-1] == 0xBB && CRC8Calc(data+3,commandLength) == data[size-2])//通过帧头帧尾和CRCC8校验
        {
            // memcpy(&command,data+3,commandLength);
            // memcpy(&rxData,data,size);
            // dataLength = size;
            command_type = static_cast<CommandType>(data[1]);
            switch (command_type)
            {
                case CommandType::NONE:
                    break;
                case CommandType::MOVE_CHASSIS:
                    memcpy(&chassis_vel,data+3,12);
                    break;
                case CommandType::MOVE_MANIPULATOR:
                    memcpy(&manipulator_angle,data+3,28);
                    break;
                case CommandType::ODOMETRY_OFFSET:
                    memcpy(&offset_data,data+3,12);
                    break;
                case CommandType::CHASSIS_STOP:
                    break;
                case CommandType::SET_PATH_POINT:
                    memcpy(&path_point,data+3,28);
            }
        }
    }

    struct ChassisVel{
        float fbVel{0};
        float lrVel{0};
        float rtvel{0};
    }__packed chassis_vel;

    struct PathPoint{
        float x{0};
        float y{0};
        float yaw{0};
        float xVel{0};
        float yVel{0};
        float rtVel{0};
        float time{0};
    }__packed path_point;

    struct ManipulatorAngle{
        float angleA{0};
        float angleB{0};
        float angleC{0};
        float angleD{0};
        float angleE{0};
        float angleF{0};
        float angleG{0};
    }__packed manipulator_angle;

    struct OffsetData{
        float x{0};
        float y{0};
        float yaw{0};
    }__packed offset_data;


private:
    // uint8_t rxData[37]{};
    // uint8_t command[32]{};
    // uint32_t dataLength{0};

    FineSerial()
    {
        HALInit::GetInstance();
        HAL_UARTEx_ReceiveToIdle_IT(uartHandleList[busID], UARTBaseLite<busID>::GetInstance().rxBuffer[0], 200);

        std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
            GetInstance().Decode(data, length);
        };
        UARTBaseLite<busID>::GetInstance().Bind(decodeFunc);
    }
    void Handle() override
    {}
};


#endif //FINESERIAL_H
