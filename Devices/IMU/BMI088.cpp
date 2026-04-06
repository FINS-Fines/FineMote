/******************************************************************************
 * Copyright (c) 2024.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "BMI088.h"
#include "main.h"

// ============ Init-only 寄存器读写（含 HAL_Delay） ============

void BMI088::AccelWriteReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    uint8_t rx[2] = {0};
    accelAgent.TransmitReceive(tx, rx, 2);
    HAL_Delay(2);
}

uint8_t BMI088::AccelReadReg(uint8_t reg) {
    uint8_t tx[3] = {static_cast<uint8_t>(reg | 0x80), 0x00, 0x00};
    uint8_t rx[3] = {0};
    accelAgent.TransmitReceive(tx, rx, 3);
    HAL_Delay(1);
    return rx[2];
}

void BMI088::GyroWriteReg(uint8_t reg, uint8_t val) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg & 0x7F), val};
    uint8_t rx[2] = {0};
    gyroAgent.TransmitReceive(tx, rx, 2);
    HAL_Delay(2);
}

uint8_t BMI088::GyroReadReg(uint8_t reg) {
    uint8_t tx[2] = {static_cast<uint8_t>(reg | 0x80), 0x00};
    uint8_t rx[2] = {0};
    gyroAgent.TransmitReceive(tx, rx, 2);
    HAL_Delay(1);
    return rx[1];
}

// ============ 初始化 ============

uint8_t BMI088::Init() {
    HAL_Delay(50);

    AccelReadReg(0x00);        // BMI088 加速度计首次读取需要 dummy read
    HAL_Delay(2);

    if (AccelReadReg(0x00) != 0x1E) return 0;  // Chip ID
    if (GyroReadReg(0x00)  != 0x0F) return 0;  // Chip ID

    AccelWriteReg(0x7D, 0x04); // ACC_PWR_CTRL: 开启加速度计
    HAL_Delay(50);
    AccelWriteReg(0x7C, 0x00); // ACC_PWR_CONF: active mode
    HAL_Delay(10);
    AccelWriteReg(0x41, 0x02); // ACC_RANGE: ±12g
    AccelWriteReg(0x40, 0xAC); // ACC_CONF: ODR 1600Hz, OSR4

    GyroWriteReg(0x0F, 0x00);  // GYRO_RANGE: ±2000°/s
    GyroWriteReg(0x10, 0x00);  // GYRO_BANDWIDTH: ODR 2000Hz

    return 1;
}

// ============ 运行时异步读取 ============

void BMI088::ReadStart() {
    accel_tx_[0] = static_cast<uint8_t>(0x12 | 0x80);
    for (int i = 1; i < 8; i++) accel_tx_[i] = 0x00;
    accelAgent.TransmitReceive(accel_tx_, accel_rx_, 8);

    gyro_tx_[0] = static_cast<uint8_t>(0x02 | 0x80);
    for (int i = 1; i < 7; i++) gyro_tx_[i] = 0x00;
    gyroAgent.TransmitReceive(gyro_tx_, gyro_rx_, 7);

    temp_tx_[0] = static_cast<uint8_t>(0x22 | 0x80);
    temp_tx_[1] = 0x00;
    accelAgent.TransmitReceive(temp_tx_, temp_rx_, 2);
}

void BMI088::Parse() {
    int16_t ax = static_cast<int16_t>((accel_rx_[3] << 8) | accel_rx_[2]);
    int16_t ay = static_cast<int16_t>((accel_rx_[5] << 8) | accel_rx_[4]);
    int16_t az = static_cast<int16_t>((accel_rx_[7] << 8) | accel_rx_[6]);

    accel[0] = ax * ACCEL_SENSITIVITY;
    accel[1] = ay * ACCEL_SENSITIVITY;
    accel[2] = az * ACCEL_SENSITIVITY;

    int16_t gx = static_cast<int16_t>((gyro_rx_[2] << 8) | gyro_rx_[1]);
    int16_t gy = static_cast<int16_t>((gyro_rx_[4] << 8) | gyro_rx_[3]);
    int16_t gz = static_cast<int16_t>((gyro_rx_[6] << 8) | gyro_rx_[5]);

    gyro[0] = gx * GYRO_SENSITIVITY;
    gyro[1] = gy * GYRO_SENSITIVITY;
    gyro[2] = gz * GYRO_SENSITIVITY;

    float bmi088_raw_temp = (int16_t)((temp_rx_[0] << 3) | (temp_rx_[1] >> 5));

    if (bmi088_raw_temp > 1023)
    {
        bmi088_raw_temp -= 2048;
    }

    temperature = bmi088_raw_temp * 0.125f + 23.0f;
}

void BMI088::TempCtrl()
{
    // TODO: 控制温度达到恒定值
}
