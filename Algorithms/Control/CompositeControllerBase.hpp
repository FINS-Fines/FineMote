#ifndef CONTROLLERTEST_COMPOSITECONTROLLERBASE_HPP
#define CONTROLLERTEST_COMPOSITECONTROLLERBASE_HPP

#include "ImplementControlBase.hpp"
#include "Observer.hpp"
#include <type_traits>
#include <array>

// trait：检测类型是否看起来像 ImplementControllerBase 的实例化
template<typename T, typename = void>
struct is_implement_controller_instantiation : std::false_type {};

template<typename T>
struct is_implement_controller_instantiation<T, std::void_t<
    decltype(T::target_size),
    decltype(T::feedback_size),
    decltype(T::control_size)
>> : std::conjunction<std::is_base_of<ControllerBase, T>> {};

// trait：检测类型是否看起来像 ObserverBase 的实例化
template<typename T, typename = void>
struct is_observer_instantiation : std::false_type {};

template<typename T>
struct is_observer_instantiation<T, std::void_t<
    decltype(T::Obs_size),
    decltype(T::Status_size),
    decltype(T::control_size)
>> : std::true_type {};

// CompositeControllerBase: composes a controller (Ctrl) and an observer (Obs)
// Template parameters: Ctrl must be an ImplementControllerBase instantiation; Obs must be an ObserverBase instantiation.
// The composite itself behaves like an ImplementControllerBase with:
//  - target_size = Ctrl::target_size
//  - feedback_size = Obs::Obs_size
//  - control_size = Ctrl::control_size

template<typename Ctrl, typename Obs>
class CompositeControllerBase : public ImplementControllerBase<
    Ctrl::target_size,
    Obs::Obs_size,
    Ctrl::control_size
> {
    using Base = ImplementControllerBase<Ctrl::target_size, Obs::Obs_size, Ctrl::control_size>;
public:
    static_assert(is_implement_controller_instantiation<Ctrl>::value,
                  "Ctrl must be an instantiation of ImplementControllerBase-like type");
    static_assert(is_observer_instantiation<Obs>::value,
                  "Obs must be an instantiation of ObserverBase-like type");

    // 检查维度是否匹配
    static_assert(Obs::Status_size == Ctrl::feedback_size,
                  "Observer's status size must equal Controller's feedback size");
    static_assert(Obs::control_size == Ctrl::control_size,
                  "Observer's control size must equal Controller's control size");

    // 添加静态成员定义，使CompositeControllerBase能够被正确识别为实现型控制器
    static constexpr size_t target_size = Ctrl::target_size;
    static constexpr size_t feedback_size = Obs::Obs_size;
    static constexpr size_t control_size = Ctrl::control_size;

    CompositeControllerBase() {
        SetupConnections();
    }

    // 构造时为内部 ctrl/obs 传入构造参数并完成连接：
    template<typename... CArgs, typename... OArgs>
    CompositeControllerBase(std::tuple<CArgs...> ctrlArgs, std::tuple<OArgs...> obsArgs)
        : ctrl(std::make_from_tuple<Ctrl>(std::move(ctrlArgs))),
          obs(std::make_from_tuple<Obs>(std::move(obsArgs)))
    {
        SetupConnections();
    }

    // 复制构造函数：在复制时重新建立内部连接
    CompositeControllerBase(const CompositeControllerBase& other)
        : ctrl(other.ctrl),  // 复制控制器
          obs(other.obs)     // 复制观察器
    {
        SetupConnections();  // 重新建立内部连接
    }

    // 同时为复合控制器与内部控制器设置目标值
    void SetTargets(std::array<float, Ctrl::target_size>& sourceOutputs) {
        Base::SetTargets(sourceOutputs);
        ctrl.SetTargets(sourceOutputs);
    }

    template<typename... Args>
    void SetTargets(Args... args) {
        static_assert(sizeof...(args) == Ctrl::target_size,
                      "Number of target pointers must match controller's target size.");
        Base::SetTargets(args...);
        ctrl.SetTargets(args...);
    }

    // 为复合控制器设置反馈的同时，也为内部观察器设置观测值
    void SetFeedbacks(const std::array<float*, Obs::Obs_size>& arr) {
        Base::SetFeedbacks(arr);
        // Obs expects SetObs for observations
        obs.SetObs(arr);
    }

    template<typename... Args>
    void SetFeedbacks(Args... args) {
        static_assert(sizeof...(args) == Obs::Obs_size,
                      "Number of feedback/observation pointers must match composite's feedback size.");
        Base::SetFeedbacks(args...);
        obs.SetObs(args...);
    }

    // 把内部控制器的outputs输出，作为复合控制器的outputs
    ControllerOutputData GetCurrentOutputs() override {
        return ctrl.GetCurrentOutputs();
    }

    ControllerOutputData GetTotalOutputs() override
    {
        return ctrl.GetTotalOutputs();
    }

    // 先计算obs，再计算ctrl，最后把ctrl计算结果同步到复合控制器的outputs
    void PerformCalc() override {
        obs.PerformCalc();

        ctrl.PerformCalc();

        auto outs = ctrl.GetCurrentOutputs();
        for (size_t i = 0; i < Ctrl::control_size; ++i) {
            this->outputs[i] = outs.dataPtr[i];
        }
        // 每次performCalc，都需要把ctrl.outputs同步到this->outputs，会带来额外开销
        // 这是由于在ControllerChain的SetConnections时，把this->outputs设置为了next的target
        // 因此，每次performCalc，都需要把ctrl.outputs同步到this->outputs，才能确保next的target是最新的
        // TODO: 能否对CompositeControllerBase的实现进行修改，避免每次performCalc都需要同步一次outputs？
    }

    void SetupConnections()
    {
        // ctrl.outputs -> obs.ctrl
        auto co = ctrl.GetCurrentOutputs();
        // co.dataPtr points to Ctrl's internal outputs array
        std::array<float*, Ctrl::control_size> ctrlPtrArr{};
        for (size_t i = 0; i < Ctrl::control_size; ++i) {
            ctrlPtrArr[i] = co.dataPtr + i;
        }
        obs.SetCtrls(ctrlPtrArr);

        // obs.status -> ctrl.feedback
        auto sd = obs.GetStatus();
        std::array<float*, Ctrl::feedback_size> fbPtrArr{};
        for (size_t i = 0; i < Ctrl::feedback_size; ++i) {
            fbPtrArr[i] = sd.dataPtr + i;
        }
        ctrl.SetFeedbacks(fbPtrArr);

        // 同步初始输出，保证在链路建立后下游指向的 composite.outputs 与 ctrl 的初始输出一致
        for (size_t i = 0; i < Ctrl::control_size; ++i) {
            this->outputs[i] = co.dataPtr[i];
        }
     }

protected:
    Ctrl ctrl;
    Obs obs;
};

#endif // CONTROLLERTEST_COMPOSITECONTROLLERBASE_HPP