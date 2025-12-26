//
// Created by wzj on 25-9-29.
//

#ifndef FINEMOTE_BM_GROUPER_H
#define FINEMOTE_BM_GROUPER_H

#include "Devicebase.h"
#include "Bus/CAN_Base.hpp"

template<int busID>
class BM_grouper : public DeviceBase {
public:
    BM_grouper(uint32_t _pub_addr, uint8_t _valid_mask) :
            canAgent(_pub_addr), valid_mask(_valid_mask) {}

    void Handle() final {
        if ((valid_mask & valid_cache) == valid_mask) { //所有定义过的电机都更新了指令
            for(uint8_t i = 0; i < 7; i++){
                canAgent[i] = group_buf[i];
            }
            canAgent.Transmit(canAgent.addr);
        } else { //调试用
            canAgent[0] = valid_cache;
            canAgent[1] = valid_mask;
            canAgent.Transmit(0x100);
        }
        valid_cache = 0;
        for (unsigned char &i: group_buf) {
            i = 0;
        }
    }

    void AddMessage(uint8_t data, uint8_t index) {
        if (index < 8) {
            group_buf[index] = data;
            valid_cache |= (0b1 << (7 - index)); //置位有效数据标识
        }
    }

    CAN_Agent<busID> canAgent;

private:
    const uint8_t valid_mask;
    uint8_t valid_cache;
    uint8_t group_buf[8] = {0};
};


#endif //FINEMOTE_BM_GROUPER_H
