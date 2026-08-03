/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CRC_HPP
#define FINEMOTE_CRC_HPP

#include "etl/crc.h"
#include "TMP/FineMote_TMP.hpp"
// #include "TMP/BSP_CRC.hpp"
// 前向声明 BSP_CRC 模板
template<typename T>
struct BSP_CRC;

/**
 * @brief 内部实现命名空间（implementation details）
 *
 * 该命名空间包含仅在本翻译单元/头文件内部使用的辅助类型和 trait。
 * 目前主要用于检测是否为给定的 ETL CRC 类型提供了 BSP 优化实现：
 * - has_bsp_crc_for<T> : 一个 type-trait，若存在可用的 BSP_CRC<T> 则为 true。
 *
 * 说明：此命名空间不属于公共 API，外部代码不应直接依赖其中符号，
 * 本意是作为实现细节供本文件中的条件选择（BSP 实现 vs ETL 实现）。
 */

/* CRC 参数说明（中文）：
 *
 * crc8_maxim（又名 Dallas/Maxim）：
 *   - @多项式: 0x31    （x^8 + x^5 + x^4 + 1）
 */
using crc8_maxim_t = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc8_maxim>>::value,
                                        BSP_CRC<etl::crc8_maxim>,
                                        etl::crc8_maxim>;
 /* crc8_ccitt:
 *   - @多项式: 0x31    （x^8 + x^2 + x + 1）
 */
using crc8_ccitt = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc8_ccitt>>::value,
                                     BSP_CRC<etl::crc8_ccitt>,
                                     etl::crc8_ccitt>;
 /* crc16_modbus:
 *   - @多项式: 0x8005    （x^16 + x^15 + x^2 + 1）
 */
using crc16_modbus = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc16_modbus>>::value,
                                       BSP_CRC<etl::crc16_modbus>,
                                       etl::crc16_modbus>;
 /* crc32（ISO 3309 / PKZIP / Ethernet）：
 *   - @多项式: 0x04C11DB7    (x^32 + x^26 + x^23 + x^22 + x^16 + x^12 + x^11 + x^10 + x^8 + x^7 + x^5 + x^4 + x^2 + x + 1)
 */
using crc32 = std::conditional_t<FineMote_TMP::is_complete<BSP_CRC<etl::crc32>>::value,
                                BSP_CRC<etl::crc32>,
                                etl::crc32>;

/**
 * @brief 使用指定的 ETL CRC 类型计算数据的 CRC 值。
 *
 * @tparam CRCx CRC类型。
 * @tparam T 数据长度的整数类型（必须为无符号整数类型）。
 * @param data 指向输入字节数组的指针。
 * @param length 输入数据的字节长度，需要是unsigned integer类型。
 * @return 由 CRCx::value() 返回的 CRC 值（类型依赖于 CRCx）。
 * @note 本函数内部使用了 `static CRCx crc;` 来复用计算器实例以减少构造开销，
 *       因此在多线程环境下该实现不是线程安全的。如果需要线程安全，请将
 *       CRC 实例改为局部非 static 变量或在外部创建并序列化访问。
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
    return crc.Calc(data, length);
}
#endif
