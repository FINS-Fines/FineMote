/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_IS_COMPLETE_HPP
#define FINEMOTE_IS_COMPLETE_HPP

#include <type_traits>

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
