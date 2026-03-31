/*******************************************************************************
* Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_STATE_SNAPSHOT_HPP
#define FINEMOTE_STATE_SNAPSHOT_HPP

#include <atomic>
#include <array>
#include <type_traits>

template <typename T>
class StateSnapshot
{
public:
    static_assert(std::is_trivially_copyable_v<T>,
                  "T must be trivially copyable");
    static_assert(std::is_standard_layout_v<T>,
                  "T must have standard layout");

    StateSnapshot() = default;

    template <typename U>
    explicit StateSnapshot(const U& initialValue)
    {
        commit(initialValue);
    }

    void Commit(const T& newValue)
    {
        T* inactiveBuffer = (committedPtr_.load(std::memory_order_relaxed) == &buffers_[0])
                                ? &buffers_[1]
                                : &buffers_[0];

        *inactiveBuffer = newValue;

        const T* expected = committedPtr_.load(std::memory_order_relaxed);
        while (!committedPtr_.compare_exchange_weak(
            expected,
            inactiveBuffer,
            std::memory_order_release,
            std::memory_order_relaxed
        ))
        {
        }
    }

    T Read() const
    {
        const T* ptr = committedPtr_.load(std::memory_order_acquire);
        return *ptr;
    }

    const T* GetPtr() const
    {
        return committedPtr_.load(std::memory_order_acquire);
    }

private:
    alignas(64) std::array<T, 2> buffers_ = {};
    alignas(64) std::atomic<const T*> committedPtr_{&buffers_[0]};
};

#endif
