//
// Created by wfrfred on 10/13/2025.
//
#include "DeviceScheduler.hpp"
#include "DeviceBase.hpp"

#include <etl/algorithm.h>
#include <FreeRTOS_POSIX/time.h>

namespace {
    void AddTicksToTimespec(timespec &time, const uint32_t ticks) {
        const int64_t nanoseconds = static_cast<int64_t>(ticks) * NANOSECONDS_PER_TICK;

        time.tv_sec += nanoseconds / NANOSECONDS_PER_SECOND;
        time.tv_nsec += static_cast<long>(nanoseconds % NANOSECONDS_PER_SECOND);

        if (time.tv_nsec >= NANOSECONDS_PER_SECOND) {
            time.tv_sec += time.tv_nsec / NANOSECONDS_PER_SECOND;
            time.tv_nsec %= NANOSECONDS_PER_SECOND;
        }
    }
} // namespace

void DeviceScheduler::RegisterDevice(DeviceBase *device) {
    if (running_) {
        Error_Handler();
    }

    const auto period = device->divisionFactor;

    const auto it = etl::find_if(buckets_.begin(), buckets_.end(), [period](const Bucket &b) {
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

    etl::sort(buckets_.begin(), buckets_.end(), [](const Bucket &a, const Bucket &b) {
        return a.period < b.period;
    });

    int priority = MAX_SCHEDULER_PRIORITY;

    for (Bucket &bucket: buckets_) {
        sched_param param{};
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

[[noreturn]] void *DeviceScheduler::BucketThreadFunc(void *arg) {
    auto *bucket = static_cast<Bucket *>(arg);
    auto &devices = bucket->devices;

    const uint32_t period = bucket->period;
    timespec next_wake_time{};
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

        clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME, &next_wake_time, nullptr);
    }
}
