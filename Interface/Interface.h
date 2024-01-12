/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_INTERFACE_H
#define FINEMOTE_INTERFACE_H

#define INRANGE(NUM, MIN, MAX) \
{\
    if(NUM<MIN){\
        NUM=MIN;\
    }else if(NUM>MAX){\
        NUM=MAX;\
    }\
}

#ifdef __cplusplus
extern "C" {
#endif
    void Setup();
    void Loop();
#ifdef __cplusplus
};
#endif

#endif //FINEMOTE_INTERFACE_H
