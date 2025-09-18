/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_DEVICEBASE_HPP
#define FINEMOTE_DEVICEBASE_HPP

#include <list>
#include "ProjectConfig.h"

class DeviceBase {
public:
    virtual void Handle() = 0;
    virtual void Update() {}
    static void DevicesHandle();

    DeviceBase();
    virtual ~DeviceBase();

    /**
     * 惰性初始化，避免静态变量初始化顺序问题
     * @return
     */
    static std::list<DeviceBase*>& getDeviceList() {
        static std::list<DeviceBase*> deviceList;
        return deviceList;
    }

    /**
     * 设置分频系数，使设备的执行频率动态可调
     * @param divisionFactor
     */
    inline void SetDivisionFactor(uint32_t divisionFactor);

protected:
    uint32_t divisionFactor = 1;

private:
    uint32_t cnt = 0;
    bool updated = 0;
};

// Constructor
inline DeviceBase::DeviceBase() {
    getDeviceList().push_back(this);
}

// Destructor
inline DeviceBase::~DeviceBase() {
    getDeviceList().remove(this);
}

// Member function definition
inline void DeviceBase::DevicesHandle() {
    static uint32_t stamp = HAL_GetTick();
    static uint32_t cnt = 0;
    uint32_t diff = HAL_GetTick() - stamp;
    cnt++;
    if (diff >= 1000) {
        //baseFre = cnt;
        stamp = HAL_GetTick();
        cnt = 0;
    }

    for (auto it = getDeviceList().begin(); it != getDeviceList().end(); ++it) {
        auto devicePtr = *it;
        if (++(devicePtr->cnt) >= devicePtr->divisionFactor) {
            devicePtr->Update();
            devicePtr->updated = 1;
            devicePtr->cnt = 0;
        }
    }

    for (auto rit = getDeviceList().rbegin(); rit != getDeviceList().rend(); ++rit) {
        auto devicePtr = *rit;
        if(devicePtr->updated) {
            devicePtr->Handle();
            devicePtr->updated = 0;
        }

    }
}

inline void DeviceBase::SetDivisionFactor(uint32_t divisionFactor) {
    this->divisionFactor = divisionFactor;
}

#endif // FINEMOTE_DEVICEBASE_HPP