//
// Created by gan_ju on 25-2-10.
//

#ifndef BMI088_H
#define BMI088_H

#define BMI088_SPI hspi2
#define BMI088_ACC_GPIOx GPIOB
#define BMI088_ACC_GPIOp GPIO_PIN_1
#define BMI088_GYRO_GPIOx GPIOB
#define BMI088_GYRO_GPIOp GPIO_PIN_0

#include "DeviceBase.h"
#include "main.h"
#include "ProjectConfig.h"
#include "BMI088reg.h"

typedef struct acc_raw_data_t {
    float x;
    float y;
    float z;
} acc_raw_data_t;

typedef struct gyro_raw_data_t {
    float roll;
    float pitch;
    float yaw;
} gyro_raw_data_t;

typedef struct acc_data_t {
    acc_raw_data_t acc_raw_data;
    float sensor_time;
    float temperature;
    bool enable_self_test;
} acc_data_t;

typedef struct gyro_data_t {
    gyro_raw_data_t gyro_raw_data;
    bool enable_self_test;
} gyro_data_t;

typedef enum bmi088_error_e {
    NO_ERROR = 0,
    ACC_CHIP_ID_ERR = 0x01,
    ACC_DATA_ERR = 0x02,
    GYRO_CHIP_ID_ERR = 0x04,
    GYRO_DATA_ERR = 0x08,
} bmi088_error_e;

typedef struct bmi088_data_t {
    acc_data_t acc_data;
    bmi088_error_e bmi088_error;
} bmi088_data_t;

class BMI088{
public:
    // 基础函数
    void WriteDataToAcc(uint8_t addr, uint8_t data);
    void WriteDataToGyro(uint8_t addr, uint8_t data);
    void ReadSingleDataFromAcc(uint8_t addr, uint8_t *data);
    void ReadSingleDataFromGyro(uint8_t addr, uint8_t *data);
    void ReadMultiDataFromAcc(uint8_t addr, uint8_t len, uint8_t *data);
    void ReadMultiDataFromGyro(uint8_t addr, uint8_t len, uint8_t *data);

    // 初始化函数
    bmi088_error_e BMI088_INIT();
    void BMI088_CONF_INIT();

    // 功能函数
    void ReadAccData(acc_raw_data_t *data);
    void ReadGyroData(gyro_raw_data_t *data);
    void ReadAccSensorTime(float *time);
    void ReadAccTemperature(float *temp);

    // 校验函数
    bmi088_error_e VerifyAccChipID();
    bmi088_error_e VerifyGyroChipID();
    bmi088_error_e VerifyAccSelfTest();
    bmi088_error_e VerifyGyroSelfTest();
};



