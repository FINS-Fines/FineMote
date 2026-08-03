/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/


#ifndef MC_BOARD_BSP_CRC_HPP
#define MC_BOARD_BSP_CRC_HPP
// BSP_CRC.hpp
// BSP_CRC.hpp
#pragma once

#include <etl/crc.h>
#include "stm32f4xx_hal.h"  // 根据你的 MCU 调整

// 前向声明 BSP_CRC 模板
template<typename T>
struct BSP_CRC;

// 独立的位反转函数（放在 namespace 中避免污染全局）
namespace detail {
template<typename T>
T reverse_bits(T val) {
    T result = 0;
    for (int i = 0; i < sizeof(T) * 8; ++i) {
        result = (result << 1) | (val & 1);
        val >>= 1;
    }
    return result;
}
}

// 为 etl::crc16_modbus 提供硬件特化
template<>
struct BSP_CRC<etl::crc16_modbus> {
    using params = etl::private_crc::crc_parameters<
        unsigned short,
        32773,   // CRC16_MODBUS 的多项式
        65535,   // INIT 值
        0,       // XOR_Out
        true     // REFIN/REFOUT
    >;

    BSP_CRC() {
        __HAL_RCC_CRC_CLK_ENABLE();
        reset();
    }

    void reset() {
        CRC->CR |= CRC_CR_RESET;
        // 如果需要自定义初始值，可以在这里补偿
    }

    void add(const uint8_t* data, const uint8_t* end) {
        while (data != end) {
            uint32_t val = *data++;
            if constexpr (params::Reflect) {
                val = detail::reverse_bits<uint32_t>(val);
            }
            CRC->DR = val;
        }
    }

    uint16_t value() const {
        uint32_t result = CRC->DR;
        if constexpr (params::Reflect) {
            result = detail::reverse_bits<uint32_t>(result);  // ← 调用独立函数，无需 const
        }
        result ^= params::Xor_Out;  // ← 修正为 Xor_Out
        return static_cast<uint16_t>(result & 0xFFFF);
    }
};
#endif //MC_BOARD_BSP_CRC_H
