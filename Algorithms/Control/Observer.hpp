//
// Created by tim67 on 2026/2/12.
//

#ifndef CONTROLLERTEST_OBSERVER_HPP
#define CONTROLLERTEST_OBSERVER_HPP
#include <array>
#include <cstdint>
using StatusData = struct StatusData{
    float* dataPtr;
    uint8_t size;
};

template <
    size_t ObsSize, // 观测的维度（相当于系统输出）
    size_t ControlSize, // 控制量的维度（相当于系统输入）
    size_t StatusSize // 系统状态的维度
>
class ObserverBase
{
public:
    static constexpr size_t Obs_size = ObsSize;
    static constexpr size_t control_size = ControlSize;
    static constexpr size_t Status_size = StatusSize;

    ObserverBase() = default;
    virtual ~ObserverBase() = default;

    // 实现以下的 set 函数：
    // - 对于观测（Obs），提供从 std::array<float, ObsSize>& 获取元素地址的重载，
    //   以及接受 ObsSize 个 float* 的可变参数重载。
    void SetObs(std::array<float, ObsSize>& sourceObservations) {
        for (size_t i = 0; i < ObsSize; ++i) {
            obsPtrs[i] = &sourceObservations[i];
        }
    }

    // 重载：直接接受 std::array<float*, ObsSize>
    void SetObs(const std::array<float*, ObsSize>& sourcePtrs) {
        for (size_t i = 0; i < ObsSize; ++i) {
            obsPtrs[i] = sourcePtrs[i];
        }
    }

    template <typename... Args>
    void SetObs(Args... args) {
        static_assert(sizeof...(args) == ObsSize,
                      "Number of observation pointers must match observer's observation size.");
        obsPtrs = {args...};
    }

    // 对于控制量（Ctrls），提供接受 std::array<float*, ControlSize> 的重载（直接保存指针），
    // 以及接受 ControlSize 个 float* 的可变参数重载。
    void SetCtrls(const std::array<float*, ControlSize>& sourceCtrls) {
        for (size_t i = 0; i < ControlSize; ++i) {
            ctrlPtrs[i] = sourceCtrls[i];
        }
    }

    template <typename... Args>
    void SetCtrls(Args... args) {
        static_assert(sizeof...(args) == ControlSize,
                      "Number of control pointers must match observer's control size.");
        ctrlPtrs = {args...};
    }


    StatusData GetStatus()
    {
        return {status.data(), static_cast<uint8_t>(StatusSize)};
    }

    virtual void PerformCalc() = 0; // 执行观测器的计算过程

protected:
    std::array<float*, ObsSize> obsPtrs{};
    std::array<float*, ControlSize> ctrlPtrs{};
    std::array<float, StatusSize> status{};
};

// 示例-feedback选择器
template <size_t ObsSize, size_t CtrlSize, size_t StatusSize>
class FeedbackSelector : public ObserverBase<ObsSize, CtrlSize, StatusSize> {
public:
    FeedbackSelector() = default;
    FeedbackSelector(std::array<int, StatusSize> selected_ids) : selected_ids(selected_ids) {}
    void PerformCalc()
    {
        // 根据 selected_ids 中记录的观测索引，将对应的观测值（通过 obsPtrs）依次拷贝到 status。
        // 若索引越界或对应指针为空，则赋值为 0.0f。
        // TODO: 能否改为在编译期判定ids数组是否合法？
        for (size_t s = 0; s < StatusSize; ++s) {
            int idx = selected_ids[s];
            if (idx < 0 || static_cast<size_t>(idx) >= ObsSize) {
                this->status[s] = 0.0f;
                continue;
            }
            float* obsPtr = this->obsPtrs[static_cast<size_t>(idx)];
            this->status[s] = (obsPtr ? *obsPtr : 0.0f);
        }
    }
private:
    std::array<int, StatusSize> selected_ids{}; // 所有被选中的反馈的ID
};


#endif //CONTROLLERTEST_OBSERVER_HPP