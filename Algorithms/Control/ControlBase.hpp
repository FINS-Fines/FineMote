/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_CONTROLBASE_HPP
#define FINEMOTE_CONTROLBASE_HPP

#include <array>
#include <cstdint>


using ControllerOutputData = struct ControllerOutputData{
    float* dataPtr;
    uint8_t size;
};

class ControllerBase {
public:
    ControllerBase() = default;
    virtual ~ControllerBase() = default;

    virtual ControllerOutputData Calc() {
        PerformCalc();
        return GetOutputs();
    }

protected:

    virtual void PerformCalc() = 0;
    virtual ControllerOutputData GetOutputs() = 0; // 注：仅返回当前环的结果，不是整个链的结果！！！
    // TODO: 后续可能需要提供两个GetOutputs，一个返回当前环的结果，一个返回整个链的结果

    // std::function<ControllerOutputData()> nextCalc = nullptr;
    // 由于连接机制的更改，因此所有与nextCalc有关的代码都被移除了
    // 新的连接机制见ControllerChain.hpp
};

template<typename T, size_t M, typename... Args, size_t... I>
std::array<T, M> CreateControllersImpl(std::index_sequence<I...>, Args&&... args) {
    return { (static_cast<void>(I), T{std::forward<Args>(args)...})... };
}

template<typename T, size_t M, typename... Args>
auto CreateControllers(Args&&... args) {
    static_assert(!std::is_same<T, ControllerBase>::value, "ControllerBase is not allowed");
    static_assert(std::is_base_of<ControllerBase, T>::value, "T must be a derivative of ControllerBase.");
    //static_assert(sizeof...(Args) <= 1, "Only one parameter is allowed");
    return CreateControllersImpl<T, M>(std::make_index_sequence<M>{}, std::forward<Args>(args)...);
}

template<template<size_t> typename T, size_t M, typename... Args>
auto CreateControllers(Args&&... args) {
    constexpr size_t N = sizeof...(Args);
    return CreateControllersImpl<T<N>, M>(std::make_index_sequence<M>{}, std::forward<Args>(args)...);
}

#endif