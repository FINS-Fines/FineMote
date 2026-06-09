/**
 * @file Task.hpp
 * @brief User Task
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2026-06-03
 */

#ifndef FINEMOTE_TASK_HPP
#define FINEMOTE_TASK_HPP

#include "DeviceBase.hpp"
#include <etl/function_traits.h>
#include <etl/type_traits.h>
#include <etl/utility.h>

/**
 * @brief Detect whether a callable is a supported task body.
 *
 * @details
 * Supported task bodies are non-capturing lambda closure types with no
 * parameters and void return type. Function pointers and functors are
 * intentionally rejected.
 */
template<typename F, typename = etl::void_t<>>
struct is_task_func: etl::false_type {};

template<typename F>
struct is_task_func<F, etl::void_t<decltype(+etl::declval<typename etl::decay<F>::type>())>> {
private:
    using Fn = typename etl::decay<F>::type;
    using Traits = etl::function_traits<decltype(+etl::declval<Fn>())>;

public:
    static constexpr bool value = etl::is_class<Fn>::value && etl::is_same<typename Traits::return_type, void>::value
                                  && Traits::argument_count == 0;
};

template<typename F>
constexpr bool is_task_func_v = is_task_func<F>::value;

/**
 * @brief User Task
 *
 * @tparam F Non-capturing lambda closure type.
 *
 * @note Task instances are created through make_task()
 * Direct, copy and move construction are disabled
 */
template<typename F>
class Task final: public DeviceBase {
public:
    using Fn = typename etl::decay<F>::type;

    Task(const Task&) = delete;
    Task& operator=(const Task&) = delete;

    Task(Task&&) = delete;
    Task& operator=(Task&&) = delete;

    void Update() override {
        func_();
    }

    void Handle() override {}

private:
    explicit Task(Fn&& func, const uint32_t divisionFactor): DeviceBase{divisionFactor}, func_(etl::move(func)) {}

    Fn func_;

    template<typename G>
    friend Task<typename etl::decay<G>::type>& make_task(G&& func, uint32_t divisionFactor);
};

/**
 * @brief Create and register a user task.
 *
 * @tparam F Lambda expression type.
 * @param func Inline non-capturing lambda with signature void().
 * @param divisionFactor Scheduler division factor passed to DeviceBase.
 * @return Reference to the statically allocated task instance.
 *
 * @details
 * The expected usage is:
 * @code
 * [[maybe_unused]] static auto& my_task = make_task([] {
 *     // periodic work
 * }, 1);
 * @endcode
 *
 * The callable must be passed as an inline lambda expression. Named lambda
 * lvalues are rejected to avoid accidental reuse of the same task identity.
 *
 * @note Each lambda closure type creates at most one task. Repeated calls with
 * the same type, including calls using moved named lambda objects, return the
 * existing task and ignore the later divisionFactor.
 */
template<typename F>
Task<typename etl::decay<F>::type>& make_task(F&& func, uint32_t divisionFactor) {
    using Fn = typename etl::decay<F>::type;

    static_assert(!etl::is_lvalue_reference<F>::value, "Task function must be passed as an inline lambda expression");

    static_assert(is_task_func_v<Fn>,
                  "Task function must be a non-capturing lambda with no parameters and void return type");

    static bool constructed = false;
    if (constructed) {
        // TODO: Log repeated task construction once the logging system is ready.
    };
    constructed = true;

    static Task<Fn> task{Fn{etl::forward<F>(func)}, divisionFactor};
    return task;
}

#endif //FINEMOTE_TASK_HPP
