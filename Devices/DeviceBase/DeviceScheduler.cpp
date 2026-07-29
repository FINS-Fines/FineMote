//
// Created by wfrfred on 10/13/2025.
//
#include "DeviceScheduler.hpp"
#include "DeviceBase.hpp"

#include <etl/algorithm.h>
#include <limits>

namespace {
constexpr int64_t NANOSECONDS_PER_MICROSECOND = 1000LL;
constexpr auto MAX_TIMESPEC_SECONDS = std::numeric_limits<time_t>::max();

void AddTicksToTimespec(timespec& time, const uint32_t ticks) {
    const auto nanoseconds = static_cast<int64_t>(ticks) * NANOSECONDS_PER_TICK;
    const auto extra_seconds = static_cast<time_t>(nanoseconds / NANOSECONDS_PER_SECOND);
    const auto extra_nanoseconds = static_cast<long>(nanoseconds % NANOSECONDS_PER_SECOND);

    if (extra_seconds > 0 && time.tv_sec > MAX_TIMESPEC_SECONDS - extra_seconds) {
        Error_Handler();
    }

    time.tv_sec += extra_seconds;
    time.tv_nsec += extra_nanoseconds;

    if (time.tv_nsec >= NANOSECONDS_PER_SECOND) {
        const auto carry_seconds = static_cast<time_t>(time.tv_nsec / NANOSECONDS_PER_SECOND);
        if (time.tv_sec > MAX_TIMESPEC_SECONDS - carry_seconds) {
            Error_Handler();
        }

        time.tv_sec += carry_seconds;
        time.tv_nsec %= NANOSECONDS_PER_SECOND;
    }
}

bool HasReachedDeadline(const timespec& now, const timespec& deadline) {
    return now.tv_sec > deadline.tv_sec || (now.tv_sec == deadline.tv_sec && now.tv_nsec >= deadline.tv_nsec);
}

useconds_t TimespecDistanceToUseconds(const timespec& now, const timespec& deadline) {
    auto seconds = static_cast<uint64_t>(deadline.tv_sec - now.tv_sec);
    long nanoseconds = deadline.tv_nsec - now.tv_nsec;

    if (nanoseconds < 0) {
        --seconds;
        nanoseconds += static_cast<long>(NANOSECONDS_PER_SECOND);
    }

    const auto rounded_microseconds =
        static_cast<uint64_t>((nanoseconds + NANOSECONDS_PER_MICROSECOND - 1) / NANOSECONDS_PER_MICROSECOND);

    if (seconds == 0 && rounded_microseconds == 0) {
        return 1;
    }

    auto microseconds = seconds * MICROSECONDS_PER_SECOND;
    microseconds += rounded_microseconds;
    return static_cast<useconds_t>(microseconds);
}
} // namespace

void DeviceScheduler::RegisterDevice(DeviceBase* device) {
    if (running_) {
        Error_Handler();
    }

    const auto period = device->divisionFactor;

    const auto it = etl::find_if(buckets_.begin(), buckets_.end(), [period](const Bucket& b) {
        return b.period == period;
    });

    if (it != buckets_.end()) {
        if (!it->devices.full()) {
            it->devices.emplace_back(device);
        } else {
            Error_Handler();
        }
    } else {
        if (!buckets_.full()) {
            Bucket bucket;
            bucket.period = period;
            bucket.devices.emplace_back(device);
            buckets_.emplace_back(etl::move(bucket));
        } else {
            Error_Handler();
        }
    }
}

void DeviceScheduler::Start() {
    if (running_) {
        return;
    }

    etl::sort(buckets_.begin(), buckets_.end(), [](const Bucket& a, const Bucket& b) {
        return a.period < b.period;
    });

    int priority = MAX_SCHEDULER_PRIORITY;

    for (Bucket& bucket: buckets_) {
        sched_param param {};
        param.sched_priority = priority;

        pthread_attr_t attr;
        if (pthread_attr_init(&attr) != 0) {
            Error_Handler();
        }

        if (pthread_attr_setschedparam(&attr, &param) != 0) {
            Error_Handler();
        }

        if (pthread_attr_setdetachstate(&attr, PTHREAD_CREATE_DETACHED) != 0) {
            Error_Handler();
        }

        if (pthread_create(&bucket.thread, &attr, &DeviceScheduler::BucketThreadFunc, &bucket) != 0) {
            Error_Handler();
        }

        if (pthread_attr_destroy(&attr) != 0) {
            Error_Handler();
        }

        priority--;
    }

    running_ = true;
}

[[noreturn]] void* DeviceScheduler::BucketThreadFunc(void* arg) {
    auto* bucket = static_cast<Bucket*>(arg);
    auto& devices = bucket->devices;

    const uint32_t period = bucket->period;
    timespec next_wake_time {};
    if (clock_gettime(CLOCK_MONOTONIC, &next_wake_time) != 0) {
        Error_Handler();
    }

    while (true) {
        AddTicksToTimespec(next_wake_time, period);

        for (const auto device: devices) {
            device->Update();
            device->updated = true;
        }

        for (auto rit = devices.rbegin(); rit != devices.rend(); ++rit) {
            if (const auto device = *rit; device->updated) {
                device->Handle();
                device->updated = false;
            }
        }

        bool slept = false;
        while (true) {
            timespec now {};
            if (clock_gettime(CLOCK_MONOTONIC, &now) != 0) {
                Error_Handler();
            }

            if (HasReachedDeadline(now, next_wake_time)) {
                if (!slept) {
                    // TODO: Log device scheduler deadline miss here.
                }
                break;
            }

            const useconds_t sleep_duration = TimespecDistanceToUseconds(now, next_wake_time);
            if (sleep_duration == 0) {
                break;
            }

            //if (usleep(sleep_duration) != 0) {
            //    Error_Handler();
            //}
            if (clock_nanosleep(CLOCK_MONOTONIC,TIMER_ABSTIME,&next_wake_time,nullptr) != 0) {
                Error_Handler();
            }

            slept = true;
        }
    }
}
