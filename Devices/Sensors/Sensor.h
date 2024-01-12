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

#define SENSOR_NUM 4//水压计数量
#define SERIAL_LENGTH_MAX 50//串口最大长度

#define B02_IIC_ADDRESS 0xEC

#define MS5837_30BA_ResetCommand     0x1E                //��λ
#define	MS5837_30BA_PROM_RD 	       0xA0                //PROM��ȡ,{0XA0,0XA2,0XA4,0XA8,0XAA,0XAC,0XAE}
#define MS5837_30BA_ADC_RD           0x00                //ADC��ȡ

#define MS5837_30BA_D1_OSR256					 0x40
#define MS5837_30BA_D1_OSR512					 0x42
#define MS5837_30BA_D1_OSR1024					 0x44
#define MS5837_30BA_D1_OSR2048					 0x46
#define MS5837_30BA_D1_OSR4096					 0x48
#define	MS5837_30BA_D1_OSR_8192   	 0x48                 //16.44msת��ʱ��
#define	MS5837_30BA_D2_OSR_8192   	 0x58                 //16.44msת��ʱ��

typedef struct Sensor_Site{
    float x[SENSOR_NUM];
    float y[SENSOR_NUM];
    float z[SENSOR_NUM];
}Sensor_Site_t;


class PressureSensor: public DeviceBase,public I2C_Agent<2>{

    unsigned char MS5837_30BA_Crc4(int id);
    unsigned long MS5837_30BA_GetConversion(uint8_t command);
    uint8_t MS5837_30BA_PROM(int id);
    void MS5837_30BA_ReSet(void);
    float MS5837_30BA_GetData(int id);
    //signed int MS5837_30BA_GetTemp(int id);

    void Init_single(int id);
    void Handle_single(int id);

    void Solve_plane_3(float* data,float* h,float* x,float* y,float* z);
    void Solve_plane(float* data,float* h,float* x,float* y,float* z,int num);
    void Update_plane();//计算水面方程

    void Delay_us(uint32_t us);
    void TCA_SelectSingleChannel(uint8_t channel){
        //this->Write(ByteVector{static_cast<uint8_t>(1<<channel)});//TODO
    }
    //void get_angle(float q[4], float *yaw, float *pitch, float *roll);

    signed int dT,TEMP;
    uint32_t Cal_C[6][7];
    int32_t OFFi,SENSi,Ti;
    int64_t OFF2;
    int64_t SENS2;
    uint32_t TEMP2;
    int64_t OFF_,SENS;
    uint32_t D1_Pres,D2_Temp;
    unsigned char flag_ok[6];


    Sensor_Site_t site;

public:

    void Handle() override;

    float Temperature;
    signed int data_temp[6];
    float data_pressure_offset[6];//压强零偏值，在水上测量
    float data_pressure_raw[6];//水下压强原始值
    float data_pressure[6];//水下压强去掉零偏，即为深度值
    float data_plane[5];//水面方程Ax+By+Cz+D=0,A*A+B*B+C*C=0,5个数据分别为A、B、C、D、拉格朗日乘数
    float data_depth;//深度数据，单位cm
    float data_level[3];
    //float quat[4];
    float pitch,roll;//俯仰角、横滚角

    //uint8_t flag_Busy;
    explicit PressureSensor(uint8_t addr): I2C_Agent<2>(addr,I2C_Bus<2>::GetInstance()){
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

        //flag_Busy = 0;

        for(int i=0;i<SENSOR_NUM;++i){

            TCA_SelectSingleChannel(i+1);
            //HAL_Delay(5);
            Init_single(i);
        }

        data_plane[0]=0;
        data_plane[1]=0;
        data_plane[2]=1;
        data_plane[3]=0;
        data_plane[4]=0;

        /*quat[0]=1.0f;
        quat[1]=0.0f;
        quat[2]=0.0f;
        quat[3]=0.0f;
    */
    };

    static PressureSensor pressure_sensor;


};

class Sonar: public DeviceBase{

    uint8_t RxBuffer[SERIAL_LENGTH_MAX];
    int32_t data;
    int32_t data_offset;

public:

    void Handle() override;
    void Receive();
    Sonar();
    static Sonar sonar;

};



#endif //FINEMOTE_SENSOR_H
