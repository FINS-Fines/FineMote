//
// Created by wfrfred on 10/13/2025.
//
#include "DeviceBase.hpp"
#include "DeviceScheduler.hpp"

// Constructor
DeviceBase::DeviceBase(const uint32_t divisionFactor): divisionFactor(divisionFactor) {
    PeripheralsInit::GetInstance();
    DeviceScheduler::GetInstance().RegisterDevice(this);
}

// Destructor
DeviceBase::~DeviceBase() {}

void DeviceBase::Update() {}
