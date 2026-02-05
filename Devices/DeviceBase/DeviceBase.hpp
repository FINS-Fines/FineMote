/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_DEVICEBASE_HPP
#define FINEMOTE_DEVICEBASE_HPP

#include "ProjectConfig.h"

class DeviceBase {
public:
    virtual void Handle() = 0;

    virtual void Update();

    explicit DeviceBase(uint32_t divisionFactor = 1);

    virtual ~DeviceBase();

    friend class DeviceScheduler;

protected:
    const uint32_t divisionFactor ;

private:
    bool updated = false;
};
#endif // FINEMOTE_DEVICEBASE_HPP
