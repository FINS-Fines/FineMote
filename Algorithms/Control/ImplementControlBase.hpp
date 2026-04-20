/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_IMPLEMENTCONTROLBASE_HPP
#define FINEMOTE_IMPLEMENTCONTROLBASE_HPP

#include <array>
#include "ControlBase.hpp"

// 基础控制器模板
template<
    size_t TargetSize,  // 目标维数
    size_t FeedbackSize, // 反馈维数
    size_t ControlSize  // 控制量（输出）维数
>
class ImplementControllerBase : public ControllerBase {
public:
    static constexpr size_t target_size = TargetSize;
    static constexpr size_t feedback_size = FeedbackSize;
    static constexpr size_t control_size = ControlSize;

    ImplementControllerBase() = default;
    virtual ~ImplementControllerBase() = default;

    // 设置目标（输入）指针
    void SetTargets(std::array<float, TargetSize>& sourceOutputs) {
        for (size_t i = 0; i < TargetSize; ++i) {
            targetPtrs[i] = &sourceOutputs[i];
        }
    }


    // 可变参数设置目标
    template <typename... Args>
    void SetTargets(Args... args) {
        static_assert(sizeof...(args) == TargetSize,
                      "Number of target pointers must match controller's target size.");
        targetPtrs = {args...};
    }

    // 重载：直接接受 std::array<float*, FeedbackSize>
    void SetFeedbacks(const std::array<float*, FeedbackSize>& sourceFeedbacks) {
        for (size_t i = 0; i < FeedbackSize; ++i) {
            // 修正：直接保存传入的指针（sourceFeedbacks 已经是 float*）
            feedbackPtrs[i] = sourceFeedbacks[i];
        }
    }

    // 设置反馈指针
    template <typename... Args>
    void SetFeedbacks(Args... args) {
        static_assert(sizeof...(args) == FeedbackSize,
                      "Number of feedback pointers must match controller's Feedback size.");
        feedbackPtrs = {args...};
    }

    // 获取输出
    ControllerOutputData GetCurrentOutputs() override {
        return { outputs.data(), static_cast<uint8_t>(ControlSize) };
    }

    // 获取最内环输出
    ControllerOutputData GetTotalOutputs() override {
        return { outputs.data(), static_cast<uint8_t>(ControlSize) };
    }

protected:
    std::array<float*, TargetSize> targetPtrs{};
    std::array<float*, FeedbackSize> feedbackPtrs{};
    std::array<float, ControlSize> outputs = {0}; // 初始化
};



// 放大器示例
template <size_t K>
class Amplifier : public ImplementControllerBase<1, 1, 1> {
public:
    Amplifier() = default;

    void PerformCalc() override {
        if (this->targetPtrs[0]) {
            this->outputs[0] = *(this->targetPtrs[0]) * static_cast<float>(K);
        }
    }

    ControllerOutputData GetCurrentOutputs() override {
        return { this->outputs.data(), 1 };
    }
    ControllerOutputData GetTotalOutputs() override {
        return { this->outputs.data(), 1 };
    }
};



#endif

