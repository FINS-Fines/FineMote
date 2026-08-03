/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CRC_HPP
#define FINEMOTE_CRC_HPP

#include "etl/crc.h"
#include "TMP/FineMote_TMP.hpp"

template<typename T>
struct BSP_CRC;

/**
 * @brief crc8_maxim（又名 Dallas/Maxim）
 * @details 多项式: 0x31
 * @details （x^8 + x^5 + x^4 + 1）
 */
using crc8_maxim_t = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc8_maxim>>::value,
                                        BSP_CRC<etl::crc8_maxim>,
                                        etl::crc8_maxim>;
/**
 * @brief crc8_ccitt:
 * @details 多项式: 0x31
 * @details （x^8 + x^2 + x + 1）
 */
using crc8_ccitt = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc8_ccitt>>::value,
                                     BSP_CRC<etl::crc8_ccitt>,
                                     etl::crc8_ccitt>;
/**
 * @brief crc16_modbus:
 * @details 多项式: 0x8005
 * @details （x^16 + x^15 + x^2 + 1）
 */
using crc16_modbus = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc16_modbus>>::value,
                                       BSP_CRC<etl::crc16_modbus>,
                                       etl::crc16_modbus>;
/**
 * @brief crc32（ISO 3309 / PKZIP / Ethernet）：
 * @details 多项式: 0x04C11DB7
 * @details (x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1)
 */
using crc32 = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc32>>::value,
                                BSP_CRC<etl::crc32>,
                                etl::crc32>;

/**
 * @brief 使用指定的 ETL CRC 类型计算数据的 CRC 值。
 * @tparam CRCx CRC类型。
 * @tparam T 数据长度的整数类型（必须为无符号整数类型）。
 * @param data 指向输入字节数组的指针。
 * @param length 输入数据的字节长度，需要是unsigned integer类型。
 * @return 由 CRCx::value() 返回的 CRC 值（类型依赖于 CRCx）。
 * @note 为了线程安全，不建议构造static对象
 */
template<typename CRCx>
class CRC_t : protected CRCx {
public:
    template<typename T>
    auto Calc(const uint8_t* data, const T length) {
        static_assert(std::is_integral_v<T>, "length must be integer");
        static_assert(std::is_unsigned_v<T>, "length must be unsigned");

        this->add(data, data + length);
        auto result = this->value();
        return result;
    }
};

template<typename CRCx, typename T>
auto CRCCalc(const uint8_t* data, const T length) {
    CRC_t<CRCx> crc;
    crc.Calc(data, length);
    return crc.value();
}

#endif
