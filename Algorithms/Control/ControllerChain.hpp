//
// Created by tim67 on 2026/2/13.
//

#ifndef CONTROLLERTEST_CONTROLLERCHAIN_HPP
#define CONTROLLERTEST_CONTROLLERCHAIN_HPP
#include <type_traits>
#include <array>
#include <tuple>
#include "ImplementControlBase.hpp"
#include "CompositeControllerBase.hpp"
// 空类型标记链结束
struct EndOfChain {};

// 声明
template<typename T1, typename T2, typename... TRest, typename A1, typename A2, typename... ARest>
auto make_nested_from_type_args(A1&& a1, A2&& a2, ARest&&... arest);

// 辅助：把 std::array<float*, N> 展开为参数包并调用 next->SetFeedbacks(...)
template<typename NextT, size_t N, size_t... I>
void CopyFeedbacksImpl(NextT* nextPtr, const std::array<float*, N>& src, std::index_sequence<I...>) {
    nextPtr->SetFeedbacks(src[I]...);
}

template<typename NextT, size_t N>
void CopyFeedbacks(NextT* nextPtr, const std::array<float*, N>& src) {
    CopyFeedbacksImpl(nextPtr, src, std::make_index_sequence<N>{});
}

// 修改后的 控制器链 连接器
template<typename Controller, typename NextController = EndOfChain>
class ControllerChain : public Controller {
    static_assert(std::is_base_of<ControllerBase, Controller>::value,
                  "Controller must inherit from ControllerBase");
    // 下一级作为直接成员（若链未结束）
    static constexpr bool has_next = !std::is_same_v<NextController, EndOfChain>;
    NextController next;

public:
    // 新增：支持通过 constructor-arg tuples 构造 ControllerChain，上层可以传入两组 tuple：
    // - 第一个 tuple 用于构造当前节点（Controller）的构造参数
    // - 第二个 tuple 用于构造下一级节点（NextController）的构造参数
    template<typename... CArgs, typename... NArgs>
    ControllerChain(std::tuple<CArgs...> ctrlArgs, std::tuple<NArgs...> nextArgs)
        : Controller(std::make_from_tuple<Controller>(std::move(ctrlArgs)))
        , next([&]() -> NextController {
            if constexpr (has_next) {
                return std::make_from_tuple<NextController>(std::move(nextArgs));
            } else {
                return NextController{}; // 当 NextController 是 EndOfChain 时，返回默认构造
            }
        }()) {
        if constexpr (has_next) {
            SetupConnections();
        }
    }

    // 复制构造函数：在复制时重新建立连接关系
    ControllerChain(const ControllerChain& other)
        : Controller(other)
        , next(other.next) {
        if constexpr (has_next) {
            SetupConnections();
        }
    }

    // 赋值运算符
    ControllerChain& operator=(const ControllerChain& other) {
        if (this != &other) {
            // 复制基类部分
            Controller::operator=(other);
            if constexpr (!std::is_same_v<NextController, EndOfChain>) {
                // 复制下一级控制器
                next = other.next;

                // 重新建立连接关系
                SetupConnections();
            }
        }
        return *this;
    }




    // 设置控制器之间的连接关系
    void SetupConnections() {
        if constexpr (has_next && is_implement_controller_instantiation<Controller>::value
                      && is_implement_controller_instantiation<NextController>::value) {
            static_assert(Controller::control_size == NextController::target_size, "...");
            static_assert(Controller::feedback_size == NextController::feedback_size, "...");
            next.SetTargets(this->outputs);
            CopyFeedbacks(&next, this->feedbackPtrs);
                      }
    }


    ControllerOutputData Calc() override {
        this->PerformCalc();
        if constexpr (has_next) {
            return next.Calc(); // 静态多态调用
        } else {
            return this->GetOutputs();
        }
    }

    // 变参形式：仅在当前节点是 ImplementControllerBase 实例化时可用
    template<typename... Args, typename = std::enable_if_t<is_implement_controller_instantiation<Controller>::value>>
    void SetFeedbacks(Args... args) {
        // 给当前节点设置（若当前是实现型）
        Controller::SetFeedbacks(args...);

        // 递归传递给下一级（仅当下一级存在且为实现型）
        if constexpr (!std::is_same_v<NextController, EndOfChain>
                      && is_implement_controller_instantiation<NextController>::value) {
            next.SetFeedbacks(args...);
                      }
    }

    // std::array\<float*, N\> 形式：只在当前为实现型时启用
    template<typename C = Controller, typename = std::enable_if_t<is_implement_controller_instantiation<C>::value>>
    void SetFeedbacks(const std::array<float*, C::feedback_size>& arr) {
        Controller::SetFeedbacks(arr);
        if constexpr (!std::is_same_v<NextController, EndOfChain>
                      && is_implement_controller_instantiation<NextController>::value) {
            next.SetFeedbacks(arr);
                      }
    }

};

// 使用辅助函数创建控制器链类型
template<typename... Controllers>
struct ChainBuilder;

template<typename Last>
struct ChainBuilder<Last> {
    using type = ControllerChain<Last>;
};

template<typename First, typename... Rest>
struct ChainBuilder<First, Rest...> {
    using NextType = typename ChainBuilder<Rest...>::type;
    using type = ControllerChain<First, NextType>;
};

// 方便使用的别名模板
template<typename... Controllers>
using MakeChain = typename ChainBuilder<Controllers...>::type;

// ========== 新增：在类型层面将 ctrl + obs 相邻对合并为 CompositeControllerBase<ctrl, obs> ============

// 简单 typelist
template<typename... Ts>
struct type_list {};

// 便捷别名
template<typename List>
struct MakeChainFromList;

template<typename... Ts>
struct MakeChainFromList<type_list<Ts...>> {
    using type = MakeChain<Ts...>;
};

// 连接 type_list
template<typename L1, typename L2>
struct concat_list;

template<typename... A, typename... B>
struct concat_list<type_list<A...>, type_list<B...>> { using type = type_list<A..., B...>; };

// ========== 合并元函数：把类似 <ctrl, obs, ctrl, ctrl, obs> -> <Composite<ctrl,obs>, ctrl, Composite<ctrl,obs>> ==========

// 主模板：处理空和单个的情况，以及多个情况
template<typename... Ts>
struct MergeCtrlObs;

// 合并分支：将前两个合并为 Composite，然后继续递归处理剩余
template<typename First, typename Second, typename... Rest>
struct MergeAsComposite {
    using type = typename concat_list<
        type_list<CompositeControllerBase<First, Second>>,
        typename MergeCtrlObs<Rest...>::type
    >::type;
};

// 保留第一个，继续处理 Second 和 Rest...
template<typename First, typename Second, typename... Rest>
struct MergeAsSeparate {
    using type = typename concat_list<
        type_list<First>,
        typename MergeCtrlObs<Second, Rest...>::type
    >::type;
};

// 辅助包装器，用于从函数返回类型
template<typename T>
struct TypeWrapper { using type = T; };



// 空列表
template<>
struct MergeCtrlObs<> {
    using type = type_list<>;
};

// 单个类型
template<typename T>
struct MergeCtrlObs<T> {
    static_assert(is_implement_controller_instantiation<T>::value,
                  "Single trailing Observer without preceding controller is not allowed");
    using type = type_list<T>;
};

// 两个及以上类型
template<typename First, typename Second, typename... Rest>
struct MergeCtrlObs<First, Second, Rest...> {
private:
    static constexpr bool isCtrlObs = is_implement_controller_instantiation<First>::value &&
                                      is_observer_instantiation<Second>::value;
    static constexpr bool isCtrlCtrl = is_implement_controller_instantiation<First>::value &&
                                       is_implement_controller_instantiation<Second>::value;

    static auto select() {
        if constexpr (isCtrlObs) {
            // 合并
            return TypeWrapper<typename MergeAsComposite<First, Second, Rest...>::type>{};
        } else if constexpr (isCtrlCtrl) {
            // 保留第一个
            return TypeWrapper<typename MergeAsSeparate<First, Second, Rest...>::type>{};
        } else {
            static_assert(sizeof...(Rest) == -1,
                          "Invalid sequence: observer cannot appear before a controller");
            // 这里的返回类型实际不会使用，因为 static_assert 已经使编译失败
            return TypeWrapper<void>{};
        }
    }

public:
    using type = typename decltype(select())::type;
};

// ========== 新增：运行时构造嵌套 tuple 的辅助函数（将用户传入的对象序列变为 ControllerChain 所需的嵌套构造参数） ==========

// Traits shortcuts
template<typename T>
using is_ctrl_t = is_implement_controller_instantiation<std::decay_t<T>>;

template<typename T>
using is_obs_t = is_observer_instantiation<std::decay_t<T>>;

// 递归构造器：从类型序列和对应的构造参数序列生成 ControllerChain 所需的嵌套 tuple
// Base case: single type
template<typename T, typename ArgTuple>
auto make_nested_from_type_args(ArgTuple&& a) {
    static_assert(is_ctrl_t<T>::value, "Single trailing element must be a controller-like type");
    using DecA = std::decay_t<ArgTuple>;
    auto ctrlTuple = std::forward<ArgTuple>(a);
    return std::tuple<DecA, std::tuple<>>(std::move(ctrlTuple), std::tuple<>());
}

// Recursive case: at least two types
template<typename T1, typename T2, typename... TRest, typename A1, typename A2, typename... ARest>
auto make_nested_from_type_args(A1&& a1, A2&& a2, ARest&&... arest) {
    if constexpr (is_ctrl_t<T1>::value && is_obs_t<T2>::value) {
        using DecA1 = std::decay_t<A1>;
        using DecA2 = std::decay_t<A2>;
        auto ctrlArgsForComposite = std::tuple<DecA1, DecA2>(std::forward<A1>(a1), std::forward<A2>(a2));
        if constexpr (sizeof...(TRest) == 0) {
            return std::tuple<decltype(ctrlArgsForComposite), std::tuple<>>(std::move(ctrlArgsForComposite), std::tuple<>());
        } else {
            auto nextNested = make_nested_from_type_args<TRest...>(std::forward<ARest>(arest)...);
            return std::tuple<decltype(ctrlArgsForComposite), decltype(nextNested)>(std::move(ctrlArgsForComposite), std::move(nextNested));
        }
    } else if constexpr (is_ctrl_t<T1>::value && is_ctrl_t<T2>::value) {
        using DecA1 = std::decay_t<A1>;
        auto ctrlTuple = std::forward<A1>(a1);
        auto nextNested = make_nested_from_type_args<T2, TRest...>(std::forward<A2>(a2), std::forward<ARest>(arest)...);
        return std::tuple<DecA1, decltype(nextNested)>(std::move(ctrlTuple), std::move(nextNested));
    } else {
        static_assert(sizeof...(ARest) == -1, "Invalid sequence: observer cannot appear before a controller (type-level)");
    }
}

// ========== 新增：携带原始类型列表的链类型 ==========
template<typename... Types>
class MakeChain_t : public MakeChainFromList<typename MergeCtrlObs<Types...>::type>::type {
    using Base = typename MakeChainFromList<typename MergeCtrlObs<Types...>::type>::type;
public:

    // 变参构造函数：接受与 Types 一一对应的参数元组
    template<typename... ArgTuples, typename = std::enable_if_t<sizeof...(ArgTuples) == sizeof...(Types)>>
MakeChain_t(ArgTuples&&... args)
    : MakeChain_t(
        make_nested_from_type_args<Types...>(std::forward<ArgTuples>(args)...)
    ) {
    }

private:
    // 私有构造函数：接受 make_nested_from_type_args 生成的嵌套元组，解包后传给基类
    template<typename NestedTuple, typename = std::enable_if_t<!std::is_same_v<std::decay_t<NestedTuple>, MakeChain_t>>>
MakeChain_t(NestedTuple&& nested)
    : Base( std::get<0>(std::forward<NestedTuple>(nested)),
            std::get<1>(std::forward<NestedTuple>(nested)) ) {}
};

#endif //CONTROLLERTEST_CONTROLLERCHAIN_HPP