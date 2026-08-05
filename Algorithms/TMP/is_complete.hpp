/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_IS_COMPLETE_HPP
#define FINEMOTE_IS_COMPLETE_HPP

#include <type_traits>
/**
 * @brief FineMote_TMP 命名空间
 *
 * 提供一组编译期类型特征，用于检测类型是否为“完整类型”（complete type）。
 *
 * 主要符号：
 * - is_complete<T>：类型特征，继承自 std::true_type / std::false_type。
 * - is_complete_v<T>：布尔常量，等价于 is_complete<T>::value。
 *
 * 判定规则：
 * - 能对类型求 sizeof(T) 的类型被认为是完整类型。
 * - 函数类型被视为完整类型。
 * - void 被视为完整类型（根据库设计选择）。
 *
 * 用途示例：在模板约束或静态断言中区分前向声明的不可完成类型与可使用的完整类型。
 *
 * 示例：
 *   struct S; // 前向声明
 *   static_assert(!FineMote_TMP::is_complete_v<S>);
 *
 *   struct T { int x; };
 *   static_assert(FineMote_TMP::is_complete_v<T>);
 */
namespace FineMote_TMP {
    template<typename T, typename = void>
    struct is_complete: std::false_type {};

    template<typename T>
    struct is_complete<T, std::void_t<decltype(sizeof(T))>>: std::true_type {};

    template<typename T>
    struct is_complete<T, std::enable_if_t<std::is_function_v<T>>>: std::true_type {};

    template<>
    struct is_complete<void, void>: std::true_type {};

    template<typename T>
    inline constexpr bool is_complete_v = is_complete<T>::value;
}

#endif
