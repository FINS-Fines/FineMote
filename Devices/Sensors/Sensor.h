/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_SENSOR_H
#define FINEMOTE_SENSOR_H

#include "DeviceBase.h"
#include "Bus/I2C_Base.h"
#include "Control/PID.h"
#include <cmath>

#define SENSOR_NUM 4    //水压计数量
#define SERIAL_LENGTH_MAX 50//串口最大长度

#define TCA9548A_ADDR   0x70
#define B02_IIC_ADDRESS 0x76

#define MS5837_30BA_ResetCommand     0x1E                //��λ
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM��ȡ,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC��ȡ

#define MS5837_30BA_D1_OSR256		    0x40
#define MS5837_30BA_D1_OSR512		    0x42
#define MS5837_30BA_D1_OSR1024		    0x44
#define MS5837_30BA_D1_OSR2048		    0x46
#define MS5837_30BA_D1_OSR4096		    0x48

#define MS5837_30BA_D2_OSR256           0x58
#define MS5837_30BA_D2_OSR512           0x58
#define MS5837_30BA_D2_OSR1024          0x58
#define MS5837_30BA_D2_OSR2048          0x58
#define MS5837_30BA_D2_OSR4096          0x58


#define CALIBRATION_CYCLES 50

typedef struct Sensor_Site{
    float x[SENSOR_NUM];
    float y[SENSOR_NUM];
    float z[SENSOR_NUM];
}Sensor_Site_t;

/**
 * 单个传感器类，管理其自身的数据资源状态
 */
class PressureSensor:public DeviceBase{
    enum class DataState_e{
        EMPTY = 0,
        RAW,
        READY
    };
    DataState_e dataState = DataState_e::EMPTY;
    enum class DeviceState_e{
        START = 0,
        PROM_READ,
        CALIBRATED,
        ERROR
    };
    DeviceState_e deviceState = DeviceState_e::START;
    uint8_t D2_Temp_Raw[3] = {};
    uint8_t D1_Pres_Raw[3] = {};
    uint32_t D2_Temp{0},D1_Pres{0};
    uint8_t Cal_C_Raw[14]={};
    uint16_t Cal_C[7]={};//读取的PROM数据
    float data_pressure_offset{0};//压强零偏值，在水上测量
    float data_pressure_raw{0};//水下压强原始值
    float data_pressure{0};//水下压强去掉零偏，即为深度值

    uint8_t channelNum;
    uint32_t calibrateNum{0};

    I2C_Agent<2> sensorI2C;
    I2C_Agent<2> channelI2C;

    uint8_t CRCCheck(){
        int cnt;
        int t;
        unsigned int n_rem=0;
        unsigned char n_bit;
        unsigned char  a=0;
        unsigned char  b=0;
        unsigned short  int n_prom[8];

        for( t=0;t<7;t++)
        {
            n_prom[t]=Cal_C[t];
        }
        n_prom[0]=((n_prom[0]) & 0x0FFF);
        n_prom[7]=0;
        for (cnt = 0; cnt < 16; cnt++)
        {
            if (cnt%2==1) n_rem ^= (unsigned short) ((n_prom[cnt>>1]) & 0x00FF);
            else n_rem ^= (unsigned short) (n_prom[cnt>>1]>>8);
            for (n_bit = 8; n_bit > 0; n_bit--)
            {
                if (n_rem & (0x8000)) n_rem = (n_rem << 1) ^ 0x3000;
                else n_rem = (n_rem << 1);
            }

        }
        n_rem= ((n_rem >> 12) & 0x000F);
        a=(n_rem ^ 0x00);
        b=Cal_C[0]>>12;
        if (a==b)
        {
            return 1;
        }
        else return 0;
    }
    void ReadPROMCallback(I2C_Task_t data){
        for (int i = 0; i < 7; ++i) {
            Cal_C[i] = (uint16_t)Cal_C_Raw[2*i]<<8 | Cal_C_Raw[2*i+1];
        }
        if (CRCCheck()==1) {
            deviceState = DeviceState_e::PROM_READ;
        }else{
            deviceState = DeviceState_e::ERROR;
        }
    }
    void ReadPROM(){//TODO 记得考虑通道选择的问题
        auto ReadPROMCallback_Pack = [this](I2C_Task_t a){this->ReadPROMCallback(a);};
        sensorI2C.Write({MS5837_30BA_ResetCommand});
        sensorI2C.Delay(20);
        for (int i = 0; i < 6; ++i) {
            sensorI2C.Write({ static_cast<uint8_t >(MS5837_30BA_PROM_RD+2*i)});
            sensorI2C.Read(Cal_C_Raw+2*i,2);
            //sensorI2C.WriteRead(MS5837_30BA_PROM_RD+2*i,Cal_C_Raw+2*i,2);
        }
        sensorI2C.Write({ static_cast<uint8_t >(MS5837_30BA_PROM_RD+2*6)});
        sensorI2C.Read(Cal_C_Raw+2*6,2,ReadPROMCallback_Pack);
        //sensorI2C.WriteRead(MS5837_30BA_PROM_RD+2*6,Cal_C_Raw+2*6,2,ReadPROMCallback_Pack);
    }
    void GetRawConversionsCallback(I2C_Task_t data){
        dataState = DataState_e::RAW;
    }
    void GetRawConversions(){

        auto GetRawConversionsCallback_Pack = [this](I2C_Task_t a){this->GetRawConversionsCallback(a);};

        sensorI2C.Write({MS5837_30BA_D2_OSR1024});
        sensorI2C.Delay(10);
        sensorI2C.WriteRead(MS5837_30BA_ADC_RD,D2_Temp_Raw,3);//TODO 有人说可能有延时，自己测一下

        sensorI2C.Write({MS5837_30BA_D1_OSR1024});
        sensorI2C.Delay(10);
        sensorI2C.WriteRead(MS5837_30BA_ADC_RD,D1_Pres_Raw,3,GetRawConversionsCallback_Pack);
    }
    void DataProcess(){
        D2_Temp = (uint32_t)D2_Temp_Raw [0] * 65536 + (uint32_t)D2_Temp_Raw[1] * 256 + (uint32_t)D2_Temp_Raw[2];
        D1_Pres = (uint32_t)D1_Pres_Raw [0] * 65536 + (uint32_t)D1_Pres_Raw[1] * 256 + (uint32_t)D1_Pres_Raw[2];

        signed int dT,TEMP;
        int64_t OFF_,SENS;
        int64_t SENS2;
        int32_t OFFi,SENSi,Ti;
        int64_t OFF2;

        dT = D2_Temp - (((uint32_t) Cal_C[5]) * 256l);
        SENS = (int64_t) Cal_C[1] * 65536l + ((int64_t) Cal_C[3] * dT) / 128l;
        OFF_ = (int64_t) Cal_C[2] * 131072l + ((int64_t) Cal_C[4] * dT) / 64l;

        //TEMP = 2000l + (int64_t)(dT) * Cal_C[id][6] / 8388608LL;
        TEMP = 2000l + (int64_t)(dT) * Cal_C[6] / 8388608LL;


        if (TEMP < 2000)  // low temp
        {

            Ti = (11 * (int64_t)(dT) * (int64_t)(dT) / (34359738368LL));
            OFFi = (31 * (TEMP - 2000) * (TEMP - 2000)) / 8;
            SENSi = (63 * (TEMP - 2000) * (TEMP - 2000)) / 32;
        } else {         // high temp
            Ti = 2 * (dT * dT) / (137438953472LL);
            OFFi = (1 * (TEMP - 2000) * (TEMP - 2000)) / 16;
            SENSi = 0;
        }
        OFF2 = OFF_ - OFFi;
        SENS2 = SENS - SENSi;
        /*if(D1_Pres==0||D2_Temp==0) return last_measure[id];
        else{
            float pressure = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
            last_measure[id]=pressure;
            return pressure;

        }*/
        data_pressure_raw = ((D1_Pres * SENS2) / 2097152.0 - OFF2) / 32768.0 / 100.0;
    }
    void TCA_SelectSingleChannel(uint8_t channel){
        channelI2C.Write(ByteVector{static_cast<uint8_t>(1<<channel)});
    }


public:
    explicit PressureSensor(uint8_t _channelNum) :
    sensorI2C(B02_IIC_ADDRESS, I2C_Bus<2>::GetInstance()),
    channelI2C(TCA9548A_ADDR, I2C_Bus<2>::GetInstance()),channelNum(_channelNum) {
        SetDivisionFactor(100);

        TCA_SelectSingleChannel(channelNum);
        sensorI2C.Delay(5);
        ReadPROM();
    }

    void Handle() override{

        TCA_SelectSingleChannel(channelNum);
        if (deviceState == DeviceState_e::START)return;

        if (deviceState == DeviceState_e::PROM_READ) {
            switch (dataState) {
                case DataState_e::EMPTY:
                    GetRawConversions();
                    break;
                case DataState_e::RAW:
                    DataProcess();
                    if (calibrateNum++ < CALIBRATION_CYCLES){
                        float temp = data_pressure_raw - data_pressure_offset;
                        if(temp<100&&temp>-10)data_pressure_offset=temp/50;//TODO 这段运算看起来很奇怪
                    }else{
                        deviceState = DeviceState_e::CALIBRATED;
                    }
                    dataState = DataState_e::EMPTY;
                case DataState_e::READY://校准阶段不对外提供数据，因此不会到达这个状态
                    break;
            }
        }else if(deviceState == DeviceState_e::CALIBRATED){
            switch (dataState) {

                case DataState_e::EMPTY:
                    GetRawConversions();
                    break;
                case DataState_e::RAW:
                    DataProcess();
                    data_pressure = data_pressure_raw -data_pressure_offset;
                    dataState = DataState_e::READY;
                    break;
                case DataState_e::READY://等待消费者使用数据,需要确保访问者唯一，并加以管理
                    break;
            }
        }
    }
    bool IsReady(){
        return dataState == DataState_e::READY;
    }
    float GetData(){
        if (dataState != DataState_e::READY)return NAN;
        dataState = DataState_e::EMPTY;
        return data_pressure;
    }
};
/**
 * 传感器组，管理其共同建立的水下姿态模型
 */
class SensorGroup: public DeviceBase{

    void Solve_plane_3(float* data,float* h,float* x,float* y,float* z);
    void Solve_plane(float* data,float* h,float* x,float* y,float* z,int num);
    void Update_plane();//计算水面方程

    Sensor_Site_t site{};

    //std::vector<PressureSensor> pressureSensors;
    PressureSensor pressureSensor0{0};
    PressureSensor pressureSensor1{1};
    PressureSensor pressureSensor2{2};
    PressureSensor pressureSensor3{3};

public:
    float data_plane[5]={0,0,1,0,0};//水面方程Ax+By+Cz+D=0,A*A+B*B+C*C=0,5个数据分别为A、B、C、D、拉格朗日乘数
    float data_depth{};//深度数据，单位cm
    float pitch{},roll{};//俯仰角、横滚角
    float data_pressure[SENSOR_NUM] = {};
    float data_level[3]={};

    void Handle() override;
    SensorGroup(){
        //------TODO:4个水压计三维坐标，由测量得到，需要修改
        Sensor_Site_t _site = {
                .x={-12,12,12,-12},
                .y={-5.4,-5.4,5.4,5.4},
                .z={0,0,0,0}
                /*.x={0},
                .y={1},
                .z={2},*/
        };
        site = _site;
        //for (int i = 0; i < SENSOR_NUM; ++i) {
        //    auto tmp = pressureSensors.size();
        //    pressureSensors.emplace_back( i+1); // TODO 可能导致debug时所有传感器不可见
        //}
        SetDivisionFactor(20);
    };

    static SensorGroup sensorGroup;

};

#endif //FINEMOTE_SENSOR_H
