/**
 * @file DeviceScheduler.hpp
 * @brief static rms device scheduler
 * @author IWIN-FINS Lab, Shanghai Jiao Tong University
 * @date 2025-10-13
 */

#ifndef FINEMOTE_DEVICE_SCHEDULER_H
#define FINEMOTE_DEVICE_SCHEDULER_H
#include <etl/vector.h>

#include "BSP_POSIX.h"
#include "DeviceBase.hpp"

// -------- Configuration --------
/**
 * @def MAX_SCHEDULER_PRIORITY
 * @brief Maximum priority used by the device scheduler
 * @note Must be less than configMAX_PRIORITIES, by default set to (configMAX_PRIORITIES - 8)
 */
#ifndef MAX_SCHEDULER_PRIORITY
    #define MAX_SCHEDULER_PRIORITY (configMAX_PRIORITIES - 8)
#endif

/**
 * @def MIN_SCHEDULER_PRIORITY
 * @brief Minimum priority used by the device scheduler
 * @note Must be greater than 1 (Idle task priority) and less than MAX_SCHEDULER_PRIORITY, by default set to 8.
 */
#ifndef MIN_SCHEDULER_PRIORITY
    #define MIN_SCHEDULER_PRIORITY 8
#endif

/**
 * @def MAX_BUCKETS
 * @brief Maximum number of buckets in the device scheduler
 * @note Must be less than or equal to (MAX_SCHEDULER_PRIORITY - MIN_SCHEDULER_PRIORITY + 1), by default set to 16
 */
#ifndef MAX_BUCKETS
    #define MAX_BUCKETS BSP_MAX_BUCKETS
#endif

/**
 * @def MAX_DEVICE_NUM
 * @brief Maximum number of devices per bucket
 * @note By default, set to 32
 */
#ifndef MAX_DEVICE_NUM
    #define MAX_DEVICE_NUM 32
#endif
// --------------------------------

// ------- Static assertions -------
static_assert(
    MAX_SCHEDULER_PRIORITY < configMAX_PRIORITIES,
    "MAX_SCHEDULER_PRIORITY must be less than configMAX_PRIORITIES"
);

static_assert(MIN_SCHEDULER_PRIORITY > 1, "MIN_SCHEDULER_PRIORITY must be greater than 1(Idle task priority)");

static_assert(
    MAX_SCHEDULER_PRIORITY - MIN_SCHEDULER_PRIORITY + 1 >= MAX_BUCKETS,
    "Not enough priority levels for the number of buckets"
);
// ----------------------------------

/**
 * @struct Bucket
 * @brief Struct containing devices with the same update period
 * @note Not copyable to ensure the safety of the pthread_t member
 */
struct Bucket {
    etl::vector<DeviceBase*, MAX_DEVICE_NUM> devices;
    pthread_t thread {};
    uint32_t period {};

    Bucket() = default;

    Bucket(const Bucket&) = delete;
    Bucket& operator=(const Bucket&) = delete;

    Bucket(Bucket&& other) noexcept: devices(etl::move(other.devices)), thread(other.thread), period(other.period) {
        other.thread = pthread_t {};
    }

    Bucket& operator=(Bucket&& other) noexcept {
        if (this != &other) {
            devices = etl::move(other.devices);
            thread = other.thread;
            period = other.period;
            other.thread = pthread_t {};
        }
        return *this;
    }
};

/**
 * @class DeviceScheduler
 * @brief Static singleton class for scheduling devices
 *
 * @details
 * 1. Automatically called by DeviceBase constructor to register devices, you should never call it manually.
 * 2. Devices are only registered during the initialization phase, before Start() is called.
 * 3. Each bucket runs in its own thread with a priority based on its update period
 */
class DeviceScheduler {
public:
    /**
     * @brief Get the singleton instance of DeviceScheduler
     * @return Reference to the DeviceScheduler instance
     */
    static DeviceScheduler& GetInstance() {
        static DeviceScheduler instance;
        return instance;
    }

    DeviceScheduler(const DeviceScheduler&) = delete;
    DeviceScheduler& operator=(const DeviceScheduler&) = delete;

    /**
     * @brief Register a device to the scheduler
     * @param device Pointer to the DeviceBase instance to be registered
     * @note Will be ignored if called after Start()
     *
     * @details
     * Groups devices based on their divisionFactor into buckets.
     * New buckets are created as needed.
     */
    void RegisterDevice(DeviceBase* device);

    /**
     * @brief Start the device scheduler
     * @note After calling this function, running_ is set to true and no more devices can be registered.
     *
     * @details
     * 1. Sorts the buckets based on RMS scheduling policy
     * 2. Assigns priorities to each bucket thread
     * 3. Creates and starts a thread for each bucket
     */
    void Start();

private:
    /**
     * @brief Private constructor for singleton pattern
     */
    DeviceScheduler() = default;

    /**
     * @brief Thread function for each bucket
     * @param arg Pointer to the Bucket instance
     * @return Never returns
     *
     * @details
     * 1. call Update() via postorder traversal
     * 2. call Handle() via preorder traversal
     * 3. sleep(suspend the thread) until next period
     */
    [[noreturn]] static void* BucketThreadFunc(void* arg);

    etl::vector<Bucket, MAX_BUCKETS> buckets_ {};
    bool running_ { false };
};

#endif // FINEMOTE_DEVICE_SCHEDULER_H
