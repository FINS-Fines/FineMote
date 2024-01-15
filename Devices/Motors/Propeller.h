/*******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_PROPELLER_H
#define FINEMOTE_PROPELLER_H

#include "DeviceBase.h"
#include "Bus/I2C_Base.h"
#include "Control/PID.h"
#include <cmath>
#include "Sensors/Sensor.h"

#define PROPELLER_NUM 8//推进器数量

#define TCA9548A_ADDR   0x70
#define PCA9685_ADDR    0x40

#define PCA9685_MODE1 0x0
#define PCA9685_PRESCALE 0xFE

#define LED0_ON_L 0x6
#define LED0_ON_H 0x7
#define LED0_OFF_L 0x8
#define LED0_OFF_H 0x9

typedef struct Propeller_Component{
    float yaw;
    float pitch;
    float roll;
    float x;
    float y;
    float z;
}Propeller_Component_t;


class Propeller :public DeviceBase{
    uint8_t rxBuffer[50];
    int32_t data[8];
    Propeller_Component_t component;
    PID pitchPID;
    PID rollPID;
    PID zPID;

    I2C_Agent<2> TCAI2C;
    I2C_Agent<2> PCAI2C;

    void FloatCtrl(){

        component.pitch = pitchPID.PIDCalc(0.0 , SensorGroup::sensorGroup.pitch);//pitch分量PID控制，目标为0
        component.roll  = rollPID.PIDCalc(0.0 , SensorGroup::sensorGroup.roll);//roll分量PID控制，目标为0
        //------TODO:Target_depth为目标深度，根据需要修改，单位cm
        float Target_depth=10.0;
        component.z = zPID.PIDCalc(Target_depth , 0);//SensorGroup::pressure_sensor.data_depth);//深度PID控制

        for(int i=0;i<PROPELLER_NUM;++i){
            data[i] = 1500;
            //------TODO:以下为提供竖直方向推力的4个推进器编号，根据接线修改
            if(i==2||i==3||i==4||i==5) {
                data[i]+=component.z;
                //------TODO:控制pitch轴旋转,前后方向相反，根据接线修改推进器编号
                if(i==2||i==3) data[i]+=component.pitch;
                else data[i]-=component.pitch;
                //------TODO:控制roll轴旋转，左右方向相反，根据接线修改推进器编号
                if(i==2||i==4) data[i]+=component.roll;
                else data[i]-=component.roll;
            }
        }
    };

    float PCA_Setfreq_Freq;
    void PCA_Setfreq_Manage(I2C_Task_t data){

        PCA_Setfreq(PCA_Setfreq_Freq,data.data[0]);
    }
    void PCA_Setfreq(float freq,uint8_t _oldmode = 0)//设置PWM频率
{       static bool firstEnter = true;
        static uint8_t prescale,oldmode,newmode;
        static double prescaleval;
    oldmode = _oldmode;
    PCA_Setfreq_Freq = freq;
    auto SetfreqManage = [this](I2C_Task_t a){this->PCA_Setfreq_Manage(a);};
        if(firstEnter){// TODO 简单的翻转逻辑，可靠性不高
            prescaleval = 25000000;
            prescaleval /= 4096;
            prescaleval /= freq;
            prescaleval -= 1;
            prescale = floor(prescaleval + 0.5f);            //floor向下取整函数
            PCAI2C.WriteRead(PCA9685_MODE1, {0}, SetfreqManage);

        }else{
            newmode = (oldmode & 0x7F) | 0x10; // sleep睡眠
            PCAI2C.Write({PCA9685_MODE1,newmode});// go to sleep（需要进入随眠状态才能设置频率）
            PCAI2C.Write({PCA9685_PRESCALE,prescale});// 设置预分频系数
            PCAI2C.Write({PCA9685_MODE1,oldmode});
            PCAI2C.Write({PCA9685_MODE1, static_cast<uint8_t>(oldmode | 0xA1)}); //
        }
        firstEnter = ! firstEnter;
    }
    /*num:舵机PWM输出引脚0~15，on:PWM上升计数值0~4096,off:PWM下降计数值0~4096*/
    void PCA_Setpwm(uint8_t num, uint32_t on, uint32_t off)//占空比: off/4096
{
        PCAI2C.Write({static_cast<uint8_t >(LED0_ON_L+4*num),static_cast<uint8_t >(on)});
        PCAI2C.Write({static_cast<uint8_t >(LED0_ON_H+4*num),static_cast<uint8_t >(on>>8)});
        PCAI2C.Write({static_cast<uint8_t >(LED0_OFF_L+4*num),static_cast<uint8_t >(off)});
        PCAI2C.Write({static_cast<uint8_t >(LED0_OFF_H+4*num), static_cast<uint8_t >(off>>8)});
}

public:
    HAL_StatusTypeDef rxState = HAL_TIMEOUT;
    void TCA_SelectSingleChannel(uint8_t channel){
        TCAI2C.Write(ByteVector{static_cast<uint8_t>(1<<channel)});
    }
    Propeller() : TCAI2C(TCA9548A_ADDR,I2C_Bus<2>::GetInstance()),
                           PCAI2C(PCA9685_ADDR,I2C_Bus<2>::GetInstance()){
        //------TODO：以下为三个自由度的PID参数，需要调试
        PID_Regulator_t _pitchPID = {
                .kp = 10000,
                .ki = 0.0,
                .kd = 0.0,
                .componentKpMax = 100,
                .componentKiMax = 100,
                .outputMax = 200
        };
        pitchPID.PIDInfo = _pitchPID;

        PID_Regulator_t _rollPID = {
                .kp = 10000,
                .ki = 0.0,
                .kd = 0.0,
                .componentKpMax = 100,
                .componentKiMax = 100,
                .outputMax = 200
        };
        rollPID.PIDInfo = _rollPID;

        PID_Regulator_t _zPID = {
                .kp = 100,
                .ki = 0.0,
                .kd = 0.0,
                .componentKpMax = 50,
                .componentKiMax = 50,
                .outputMax = 100
        };
        zPID.PIDInfo = _zPID;
        SetDivisionFactor(100);
        TCA_SelectSingleChannel(4);
        PCAI2C.Write({PCA9685_MODE1, 0x0});
        PCA_Setfreq(50);//Hz
        for(int i=0;i<PROPELLER_NUM;++i){
            data[i]=1500;
            //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
            PCA_Setpwm(i,0,floor(data[i] * 4096 / 20000 + 0.5f));
        }
    }


    void Handle() override{
        if (rxState != HAL_OK) {
            rxState = HAL_UARTEx_ReceiveToIdle_IT(&Serial_Host, rxBuffer, 50);
        }
        //FloatCtrl();//PID控制悬浮状态
        TCA_SelectSingleChannel(4);
        for(int i=0;i<PROPELLER_NUM;++i){
            //------TODO：i为推进器在扩展版上的接口编号，根据接线修改，目前为0-7号
            PCA_Setpwm( i, 0, floor(data[i] * 4096 / 20000 + 0.5f) );
        }
    };
    void Trigger(){
        data_extract(rxBuffer, data, 8);
        HAL_UARTEx_ReceiveToIdle_IT(&Serial_Host, rxBuffer, 50);
    }
    static void data_extract(uint8_t *rx, int32_t *data, int32_t num){
        // 示例：假设 PWM 命令格式是 "PWM:1000,2000,1500,1800,1600,1400\n"
        if (strncmp((char*)rx, "PWM:", 4) == 0) {

            char *data_str = (char*)rx + 4;
            char *token = strtok(data_str, ",");
            int i = 0;
            while (token != NULL && i < num) {
                data[i] = atoi(token);
                token = strtok(NULL, ",");
                i++;
            }
        }
    }
};

#endif //FINEMOTE_PROPELLER_H
