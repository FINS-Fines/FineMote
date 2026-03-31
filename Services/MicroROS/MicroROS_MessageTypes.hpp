/******************************************************************************
 * Copyright (c) 2026.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#ifndef FINEMOTE_MICROROS_MESSAGE_TYPES_HPP
#define FINEMOTE_MICROROS_MESSAGE_TYPES_HPP

#include <sensor_msgs/msg/joint_state.h>
#include <nav_msgs/msg/odometry.h>
#include <std_msgs/msg/bool.h>
#include <std_msgs/msg/int32.h>

template <typename T>
struct RosMsgTraits
{
    static constexpr bool registered = false;
};

#define DEFINE_MICROROS_MSG(CppType, PkgName, MsgSub, MsgName) \
    template<> \
    struct RosMsgTraits<CppType> { \
        static constexpr bool registered = true; \
        static constexpr const char* name = #MsgName; \
        static const rosidl_message_type_support_t* GetTypeSupport() { \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName); \
        } \
    };

#define PUBLISHER(obj) RosPublisher(#obj, obj)

DEFINE_MICROROS_MSG(sensor_msgs__msg__JointState, sensor_msgs, msg, JointState)

DEFINE_MICROROS_MSG(nav_msgs__msg__Odometry, nav_msgs, msg, Odometry)

DEFINE_MICROROS_MSG(std_msgs__msg__Bool, std_msgs, msg, Bool)

DEFINE_MICROROS_MSG(std_msgs__msg__Int32, std_msgs, msg, Int32)


#endif
