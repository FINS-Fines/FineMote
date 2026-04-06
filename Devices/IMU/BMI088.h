/******************************************************************************
* Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_BMI088_H
#define FINEMOTE_BMI088_H

#include "Board.h"
#include "Bus/SPI_Base.hpp"
#include "DeviceBase/DeviceBase.hpp"

class BMI088 : public DeviceBase {
public:
    BMI088() : DeviceBase()
    {
        Init();
        ReadStart();
    }

    uint8_t Init();

    void Handle() override {
        Parse();
        ReadStart();
    }

    void Update() override
    {

    }

    void ReadStart();
    void Parse();

    void TempCtrl();

    float accel[3] = {}; // Accel单位为m/s2
    float gyro[3]  = {}; // Gyro单位为rad/s
    float temperature = 0;

private:
    void AccelWriteReg(uint8_t reg, uint8_t val);
    uint8_t AccelReadReg(uint8_t reg);
    void GyroWriteReg(uint8_t reg, uint8_t val);
    uint8_t GyroReadReg(uint8_t reg);

    static constexpr float ACCEL_SENSITIVITY = 0.003590f;
    static constexpr float GYRO_SENSITIVITY  = 0.001065f;

    uint8_t accel_tx_[8] = {};
    uint8_t accel_rx_[8] = {};
    uint8_t gyro_tx_[7]  = {};
    uint8_t gyro_rx_[7]  = {};
    uint8_t temp_tx_[2] = {};
    uint8_t temp_rx_[2] = {};

    SPI_Agent<2> accelAgent{1};
    SPI_Agent<2> gyroAgent{2};
};

#endif // FINEMOTE_BMI088_H