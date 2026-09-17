/******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/
#ifndef FINEMOTE_PLATFROM_ERROR_H
#define FINEMOTE_PLATFROM_ERROR_H

typedef enum {
    PLATFORM_OK              = 0,
    PLATFORM_GENERAL         = 1,
    PLATFORM_TIMEOUT         = 2,
    PLATFORM_INVALID_PARAM   = 3,
    PLATFORM_NO_MEMORY       = 4,
    PLATFORM_NO_RESOURCE     = 5,
    PLATFORM_NOT_SUPPORTED   = 6,
    PLATFORM_NOT_INITIALIZED = 7,
    PLATFORM_ALREADY_INIT    = 8,
    PLATFORM_BUSY            = 9,
    PLATFORM_FAIL            = 10,
    PLATFORM_RESERVED        = 0x7FFFFFFF
} PlatformErr;

#endif


