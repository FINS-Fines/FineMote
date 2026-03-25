/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include <stddef.h>

int setenv(const char* name, const char* value, int overwrite)
{
    (void)name;
    (void)value;
    (void)overwrite;
    return 0;
}

int unsetenv(const char* name)
{
    (void)name;
    return 0;
}

const char _ctype_[256] = {0};
void* _impure_ptr = NULL;
