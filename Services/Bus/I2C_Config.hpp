/******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#ifndef FINEMOTE_I2C_CONFIG_HPP
#define FINEMOTE_I2C_CONFIG_HPP

#include <cstdint>

enum class I2C_DeviceAddressWidth : uint8_t
{
    Bits7,
    Bits10
};

enum class I2C_MemoryAddressEndian : uint8_t
{
    BigEndian,
    LittleEndian
};

struct I2C_DeviceConfig
{
    I2C_DeviceAddressWidth addressWidth;
    uint16_t deviceAddress;
    uint32_t clockSpeedHz;
};

#endif // FINEMOTE_I2C_CONFIG_HPP
