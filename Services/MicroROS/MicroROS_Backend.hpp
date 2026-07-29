/*******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_BACKEND_HPP
#define FINEMOTE_MICROROS_BACKEND_HPP

#include <rmw/init_options.h>
#include <rmw/ret_types.h>

class MicroROS_Backend {
public:
    virtual ~MicroROS_Backend() = default;

    virtual bool Prepare() = 0;
    virtual rmw_ret_t Configure(rmw_init_options_t* options) = 0;
};

#endif
