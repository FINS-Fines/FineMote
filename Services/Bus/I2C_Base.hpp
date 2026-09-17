/******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_I2C_BASE_HPP
#define FINEMOTE_I2C_BASE_HPP

#include <cstddef>
#include <cstdint>

#include "BSP_POSIX.h"
#include "Bus/I2C_Config.hpp"
#include "Bus/platform_error.h"
#include "BSP_I2C.hpp"

template<size_t ID>
class I2C_Agent;


#if FINEMOTE_I2C_BASE_DISENABLED == 0
//-------------------------- bus mutex --------------------------------------//
#if FINEMOTE_BSP_I2C_HAS_INTERNAL_LOCK
#define FINEMOTE_I2C_BASE_USE_BUS_MUTEX 0
#else
#define FINEMOTE_I2C_BASE_USE_BUS_MUTEX 1
#endif

#if FINEMOTE_I2C_BASE_USE_BUS_MUTEX

class I2C_BusMutex
{
public:
    I2C_BusMutex(){
        #if !FINEMOTE_BSP_I2C_MUTEX_LAZY_INIT
        initialized_ = (pthread_mutex_init(&mutex_, nullptr) == 0);
        #endif
    }

    I2C_BusMutex(const I2C_BusMutex&) = delete;
    I2C_BusMutex& operator=(const I2C_BusMutex&) = delete;

    void lock()
    {
        #if FINEMOTE_BSP_I2C_MUTEX_LAZY_INIT
        if (!initialized_){
            initialized_ = (pthread_mutex_init(&mutex_, nullptr) == 0);
        }
        #endif
        if (initialized_){
            (void)pthread_mutex_lock(&mutex_);
        }
    }

    void unlock(){
        if (initialized_){
            (void)pthread_mutex_unlock(&mutex_);
        }
    }

private:
    pthread_mutex_t mutex_ {};
    bool initialized_ = false;
};

#else

class I2C_BusMutex
{
public:
    I2C_BusMutex() = default;
    void lock()   {}
    void unlock() {}
};

#endif
//-------------------------- lock guard -------------------------------------//
template<typename TMutex>
class I2C_LockGuard
{
public:
    explicit I2C_LockGuard(TMutex& mutex) : mutex_(mutex) { mutex_.lock(); }
    ~I2C_LockGuard() { mutex_.unlock(); }

    I2C_LockGuard(const I2C_LockGuard&) = delete;
    I2C_LockGuard& operator=(const I2C_LockGuard&) = delete;

private:
    TMutex& mutex_;
};

template<size_t ID>
class I2C_Base
{
public:
    static I2C_Base& GetInstance()
    {
        static I2C_Base instance;
        return instance;
    }

    I2C_Base(const I2C_Base&) = delete;
    I2C_Base& operator=(const I2C_Base&) = delete;

    void* CreateDeviceHandle(const I2C_DeviceConfig& config)
    {
        I2C_LockGuard<I2C_BusMutex> guard(busMutex_);
        return BSP_I2C<ID>::GetInstance().CreateDeviceHandle(config);
    }

    PlatformErr Transmit(void* deviceHandle, uint16_t size, const uint8_t* pData)
    {
        I2C_LockGuard<I2C_BusMutex> guard(busMutex_);
        return BSP_I2C<ID>::GetInstance().Transmit(deviceHandle, size, pData);
    }

    PlatformErr Receive(void* deviceHandle, uint16_t size, uint8_t* pData)
    {
        I2C_LockGuard<I2C_BusMutex> guard(busMutex_);
        return BSP_I2C<ID>::GetInstance().Receive(deviceHandle, size, pData);
    }

    PlatformErr MemWrite(void* deviceHandle,
                         uint16_t memAddr,
                         uint8_t memAddrSize,
                         I2C_MemoryAddressEndian memAddrEndian,
                         uint16_t size,
                         const uint8_t* pData)
    {
        I2C_LockGuard<I2C_BusMutex> guard(busMutex_);
        return BSP_I2C<ID>::GetInstance().MemWrite(
            deviceHandle, memAddr, memAddrSize, memAddrEndian, size, pData);
    }

    PlatformErr MemRead(void* deviceHandle,
                        uint16_t memAddr,
                        uint8_t memAddrSize,
                        I2C_MemoryAddressEndian memAddrEndian,
                        uint16_t size,
                        uint8_t* pData)
    {
        I2C_LockGuard<I2C_BusMutex> guard(busMutex_);
        return BSP_I2C<ID>::GetInstance().MemRead(
            deviceHandle, memAddr, memAddrSize, memAddrEndian, size, pData);
    }

private:
    friend class I2C_Agent<ID>;
    I2C_BusMutex busMutex_;
    I2C_Base() { BSP_I2C<ID>::GetInstance(); }
};

#endif


template<size_t ID>
class I2C_Agent
{
public:
    /**
     * @param config I2C device address and bus frequency configuration.
     */
    explicit I2C_Agent(const I2C_DeviceConfig& config)
        : deviceID(config.deviceAddress),
          deviceHandle(nullptr)
    {
        static_assert(ID > 0 && ID <= I2C_BUS_MAXIMUM_COUNT,
                      "Using illegal I2C BUS");
        I2C_Base<ID>::GetInstance();
        deviceHandle = I2C_Base<ID>::GetInstance().CreateDeviceHandle(config);
    }

    PlatformErr Transmit(const uint8_t* pTxData, uint16_t size)
    {
        if (pTxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().Transmit(deviceHandle, size, pTxData);
    }

    PlatformErr Receive(uint8_t* pRxData, uint16_t size)
    {
        if (pRxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().Receive(deviceHandle, size, pRxData);
    }

    PlatformErr MemWrite(uint16_t memAddr,
                         uint8_t memAddrSize,
                         I2C_MemoryAddressEndian memAddrEndian,
                         const uint8_t* pTxData,
                         uint16_t size)
    {
        if (pTxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().MemWrite(
            deviceHandle, memAddr, memAddrSize, memAddrEndian, size, pTxData);
    }

    //对于10位iic地址，esp-idf底层驱动时序有异常，目前issues尚未合并，不建议使用
    PlatformErr MemRead(uint16_t memAddr,
                        uint8_t memAddrSize,
                        I2C_MemoryAddressEndian memAddrEndian,
                        uint8_t* pRxData,
                        uint16_t size)
    {
        if (pRxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().MemRead(
            deviceHandle,
            memAddr,
            memAddrSize,
            memAddrEndian,
            size,
            pRxData);
    }

    //todo:下面两个函数暂时不准备支持
    //在stm32的hal库支持跨函数调用，这个要求在应用层用户加锁，而不是在驱动层加锁
    //在esp-idf库不支持跨函数调用，由于在驱动层加锁
    //函数接口不能直接使用如下接口
    /*PlatformErr SeqTransmit(const uint8_t* pTxData, uint16_t size, uint32_t xferOptions)
    {
        if (pTxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().SeqTransmit(
            deviceID, 0, size, pTxData, xferOptions);
    }

    PlatformErr SeqReceive(uint8_t* pRxData, uint16_t size, uint32_t xferOptions)
    {
        if (pRxData == nullptr || size == 0) {
            return PLATFORM_INVALID_PARAM;
        }
        return I2C_Base<ID>::GetInstance().SeqReceive(
            deviceID, 0, size, pRxData, xferOptions);
    }*/

private:
    const uint16_t deviceID;
    void* deviceHandle;
};

#endif
