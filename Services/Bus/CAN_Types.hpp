/*******************************************************************************
* Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef MC_BOARD_CAN_TYPES_H
#define MC_BOARD_CAN_TYPES_H

#include <cstddef>
#include <cstdint>

struct BxCAN {
    static constexpr std::size_t BufferSize = 8;

    struct Flags {
        uint8_t is_ext : 1;
        uint8_t is_rtr : 1;
    };
};

struct FDCAN {
    static constexpr std::size_t BufferSize = 64;

    struct Flags {
        uint8_t is_ext : 1;
        uint8_t is_rtr : 1;
        uint8_t is_fd : 1;
        uint8_t is_brs : 1;
    };
};

/**
 * Must be specialized for each CAN ID in BSP
 * should define:
 *     using CAN_Type = BxCAN or FDCAN
 * @tparam ID CAN bus ID
 */
template<uint8_t ID>
struct CAN_Traits;

template<typename CAN_Type>
struct CAN_Package {
    static constexpr std::size_t BufferSize = CAN_Type::BufferSize;
    using Flags_Type = typename CAN_Type::Flags;

    uint32_t id;
    uint8_t len;
    uint8_t data[BufferSize];

    Flags_Type flags;
};

#endif //MC_BOARD_CAN_TYPES_H
