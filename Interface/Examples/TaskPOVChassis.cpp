/*******************************************************************************
 * Copyright (c) 2025.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

#ifdef __KEIL_APP
#ifndef WITH_POV_EXAMPLE
#define WITH_POV_EXAMPLE
#endif
#endif

#ifdef WITH_POV_EXAMPLE

#include "Scheduler.h"

/**
 * Part 1 (Case 1): Motor definitions.
 */

#include "Motors/Motor4010.hpp"
#include "Motors/Motor4315.hpp"

#include "Control/PID.hpp"

constexpr PID_Param_t speedPID = {0.23f, 0.008f, 0.3f};
auto wheelControllers = CreateControllers<PID, 4>(speedPID);
auto swerveControllers = CreateControllers<Amplifier<1>, 4>();

#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);
Motor4010<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x143);
Motor4010<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x142);
Motor4010<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x141);

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position, true}
Motor4315<1> SBRMotor(DIRECT_POSITION, swerveControllers[0], 0x04, 20);
Motor4315<1> SBLMotor(DIRECT_POSITION, swerveControllers[1], 0x03, 20);
Motor4315<1> SFLMotor(DIRECT_POSITION, swerveControllers[2], 0x02, 20);
Motor4315<1> SFRMotor(DIRECT_POSITION, swerveControllers[3], 0x01, 20);




/**
 * Part 1 (Case 2): Motor definitions.
 */
//
// #include "Motors/RMD_L_40xx_v3.hpp"
//
// #include "Control/PID.hpp"
//
// constexpr PID_Param_t speedPID = {0.15f, 0.00145f, 0.0016f};
// constexpr PID_Param_t positionInnerPID = {0.5f, 0.0f, 0.02f};
// constexpr PID_Param_t positionOuterPID = {1.0f, 0.00f, 0.01f};
//
// auto wheelControllers = CreateControllers<PID, 4>(speedPID);
// auto outerControllers = CreateControllers<PID, 4>(positionOuterPID);
// auto innerControllers = CreateControllers<PID, 4>(positionInnerPID);
//
// #define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
// #define TORQUE_2_POSITION {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Position, true}
//
// RMD_L_40xx_v3<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x242);
// RMD_L_40xx_v3<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x244);
// RMD_L_40xx_v3<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x246);
// RMD_L_40xx_v3<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x248);
//
// RMD_L_40xx_v3<1> SFRMotor(TORQUE_2_POSITION, outerControllers[0], 0x241);
// RMD_L_40xx_v3<1> SFLMotor(TORQUE_2_POSITION, outerControllers[1], 0x243);
// RMD_L_40xx_v3<1> SBLMotor(TORQUE_2_POSITION, outerControllers[2], 0x245);
// RMD_L_40xx_v3<1> SBRMotor(TORQUE_2_POSITION, outerControllers[3], 0x247);




/**
 * Part 2: Chassis definitions.
 */

#include "Chassis/POV_Chassis.hpp"

constexpr float ROBOT_LENGTH = 0.240225f;
constexpr float ROBOT_WIDTH = 0.24f;
constexpr float WHEEL_DIAMETER = 0.0483f;

auto chassis = POV_ChassisBuilder<PlanarOdom>(
    WHEEL_DIAMETER,
    Swerve_t{&SFRMotor, &CFRMotor, ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, 180},
    Swerve_t{&SFLMotor, &CFLMotor, ROBOT_LENGTH / 2, ROBOT_WIDTH / 2},
    Swerve_t{&SBLMotor, &CBLMotor, -ROBOT_LENGTH / 2, ROBOT_WIDTH / 2},
    Swerve_t{&SBRMotor, &CBRMotor, -ROBOT_LENGTH / 2, -ROBOT_WIDTH / 2, 180}
);




/**
 * Part 3: Command input definitions.
 */

#include "RemoteControllers/RadioMaster_Zorro.h"
#include "FineWarden/FineSerial.hpp"


RadioMaster_Zorro remote;
UARTBuffer<3, 200> uart3Buffer([](uint8_t* data, size_t length) {
    remote.Decode(data, length);
});




/**
 * Part 4: micro-ROS definitions.
 */

#include "MicroROS/MicroROS_Agent.hpp"
#include "MicroROS/MicroROS_Manager.hpp"
#include "MicroROS/MicroROS_MessageTypes.hpp"

auto pub_chassis_twist = PUBLISHER(chassis);

int32_t count = 0;
RosPublisher pub_hb("heartbeat", [](std_msgs__msg__Int32& msg) {
    msg.data = count++;
});


RosSubscriber sub_led("cmd/led", [](const std_msgs__msg__Bool& msg)
{
    HAL_GPIO_WritePin(LED_GPIO_Port, LED_Pin, msg.data ? GPIO_PIN_RESET : GPIO_PIN_SET);
});



/**
 * Part 5: Task definitions.
 */
static bool cascaded=0;
void TaskPOVChassis() {
    constexpr float SPEED_LIMIT = 2.0f;
//    if(!cascaded){
//        outerControllers[0].Cascade(innerControllers[0]);
//        outerControllers[1].Cascade(innerControllers[1]);
//        outerControllers[2].Cascade(innerControllers[2]);
//        outerControllers[3].Cascade(innerControllers[3]);
//        cascaded=1;
//    }
    if (remote.GetInfo().sC == RemoteControl::SWITCH_STATE_E::UP_POS) {
        std::array<float, 3> targetV = {
            remote.GetInfo().rightCol * SPEED_LIMIT,
            -remote.GetInfo().rightRol * SPEED_LIMIT,
            -remote.GetInfo().leftRol * PI
        };
        chassis.SetVelocity(std::move(targetV));
    }
}
TASK_EXPORT(TaskPOVChassis);

#endif