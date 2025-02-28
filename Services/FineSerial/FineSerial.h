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


enum class SingleCommandType{
    NONE = 0X00,
    MISSION_START = 0X01,
    SET_PATH_POINT = 0X02,
    ODOMETRY_OFFSET = 0X03,
    CHASSIS_STOP = 0X04,
    END_EFFECTOR = 0X05,
};

enum class ContinuousCommandType {
    NONE = 0X00,
    MOVE_CHASSIS = 0X0A,
    MOVE_MANIPULATOR = 0X0B,
};


template <uint8_t busID>
class FineSerial : public DeviceBase{
public:
    uint8_t rxData[37]{};
    uint8_t cnt{0};
    uint8_t crc8{0};
    uint16_t datasize{0};
    uint8_t command[32]{};
    uint32_t dataLength{0};
    uint32_t delayTime{0};
    float timeMsgNotReceived{0};
    bool isCurrentTaskFinished = false;
    bool endEffectorState = true;//false关，true开
    bool isMissionStart = false;
    bool chassisStopFlag = false;
    SingleCommandType singleCommand = SingleCommandType::NONE;
    ContinuousCommandType continuousCommand = ContinuousCommandType::NONE;

    static FineSerial& GetInstance(){
        static FineSerial instance;
        return instance;
    }

    //单例模式删除拷贝构造运算符
    FineSerial(const FineSerial&) = delete;
    FineSerial& operator=(const FineSerial&) = delete;

    void AvtivateUpload(float _delayTime = 0){
        delayTime = static_cast<uint32_t>(_delayTime * 1000);
        isUploadActive = true;
        cnt++;
    }

    void UploadMsg(){
        static uint8_t cnt{0};
        cnt++;
        if (cnt >= 5){
            UARTBaseLite<5>::GetInstance().Transmit(upLoadCommand, 3);
            cnt = 0;
        }
    }

    void Decode(uint8_t* data, uint16_t size){
        lastMsgReceivedTick = HAL_GetTick();
        // memcpy(&rxData,data,size);
        // memcpy(&command,data+3,commandLength);
        // dataLength = size;
        // crc8 = data[size-2];
        if(data[0] == 0xAA && data[size-1] == 0xBB){
            uint8_t commandLength = data[2];
            if(CRC8Calc(data+3,commandLength) != data[size-2]){
                return;
            }
            if(data[1]<0x0A){
                singleCommand = static_cast<SingleCommandType>(data[1]);
                switch (singleCommand){
                case SingleCommandType::NONE:
                    break;
                case SingleCommandType::MISSION_START:
                    isMissionStart = true;
                    break;
                case SingleCommandType::SET_PATH_POINT:
                    memcpy(&path_point,data+3,12);
                    break;
                case SingleCommandType::ODOMETRY_OFFSET:
                    memcpy(&offset_data,data+3,12);
                    break;
                case SingleCommandType::CHASSIS_STOP:
                    chassisStopFlag = (data[3] == 0x01);
                    break;
                case SingleCommandType::END_EFFECTOR:
                    endEffectorState = (data[3] == 0x01);
                    break;
                }
                isCurrentTaskFinished = false;
            }
            else{
                continuousCommand = static_cast<ContinuousCommandType>(data[1]);
                switch (continuousCommand){
                case ContinuousCommandType::NONE:
                    break;
                case ContinuousCommandType::MOVE_CHASSIS:
                    memcpy(&chassis_vel,data+3,12);
                    break;
                case ContinuousCommandType::MOVE_MANIPULATOR:
                    if(size != 29){return;}
                    memcpy(&manipulator_angle,data+3,24);

                    break;
                }
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
        // float xVel{0};
        // float yVel{0};
        // float rtVel{0};
        // float time{0};
    }__packed path_point;

    struct ManipulatorAngle{
        float angleA{0};
        float angleB{0};
        float angleC{0};
        float angleD{0};
        float angleE{0};
        float angleF{0};
    }__packed manipulator_angle;

    ManipulatorAngle __packed offsetAngle;

    struct OffsetData{
        float x{0};
        float y{0};
        float yaw{0};
    }__packed offset_data;


private:
    uint8_t upLoadCommand[3]{0xAA,0x01,0xBB};
    uint32_t initTick{0};
    uint32_t lastMsgReceivedTick{0};
    float angleOffset[6]{0,0,-0.3,0,0.1,0};
    bool isUploadActive = false;
    void Upload(){
        if(singleCommand == SingleCommandType::NONE){
            return;
        }
        uint8_t txData[6]{};
        txData[0] = 0xAA;
        txData[1] = static_cast<uint8_t>(singleCommand);
        txData[2] = 0x01;
        txData[3] = static_cast<uint8_t>(isCurrentTaskFinished);
        txData[4] = 0x00;
        txData[5] = 0xBB;
        UARTBaseLite<busID>::GetInstance().Transmit(txData,6);

        isCurrentTaskFinished = false;
        singleCommand = SingleCommandType::NONE;
    }

    FineSerial(){
        HALInit::GetInstance();
        HAL_UARTEx_ReceiveToIdle_IT(uartHandleList[busID], UARTBaseLite<busID>::GetInstance().rxBuffer[0], 200);
        std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
            GetInstance().Decode(data, length);
        };
        UARTBaseLite<busID>::GetInstance().Bind(decodeFunc);
    }

    void Handle() override{
        // if(isCurrentTaskFinished)
        // {
        //     Upload();
        // }
        offsetAngle.angleA = manipulator_angle.angleA + angleOffset[0]*PI/180.f;
        offsetAngle.angleB = manipulator_angle.angleB + angleOffset[1]*PI/180.f;
        offsetAngle.angleC = manipulator_angle.angleC + angleOffset[2]*PI/180.f;
        offsetAngle.angleD = manipulator_angle.angleD + angleOffset[3]*PI/180.f;
        offsetAngle.angleE = manipulator_angle.angleE + angleOffset[4]*PI/180.f;
        offsetAngle.angleF = manipulator_angle.angleF + angleOffset[5]*PI/180.f;
        timeMsgNotReceived = 0.001f*(HAL_GetTick()-lastMsgReceivedTick);
        if(isUploadActive){
            if(delayTime > 0)
            {
                delayTime--;
                initTick = HAL_GetTick();
            }else
            {
                UploadMsg();
                isUploadActive = HAL_GetTick() - initTick < 500;
            }
        }
        else if(!isUploadActive){
            initTick = HAL_GetTick();
        }
    }
};


#endif //FINESERIAL_H
