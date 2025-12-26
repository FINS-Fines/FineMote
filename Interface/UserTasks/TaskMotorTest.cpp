//
// Created by wzj on 25-9-29.
//
#include "Scheduler.h"
#include "Motors/BM_grouper.hpp"
#include "Motors/BM1010.hpp"
#include "Motors/BM1505.hpp"
#include "Motors/DM4310.hpp"
#include "RemoteControllers/RadioMaster_Pocket.h"
#include "RemoteControllers/RadioMaster_Zorro.h"

#include "Control/PID.hpp"

#include "MultiMedia/BeepMusic.hpp"

//constexpr PID_Param_t speedPID = {0.23f, 0.008f, 0.3f};
std::array<Amplifier<1>, 2> wheelControllers = {Amplifier<1>{}};
std::array<Amplifier<1>, 2> suspensionControllers = {Amplifier<1>{}};
std::array<Amplifier<1>, 2> steerControllers = {Amplifier<1>{}};
//// auto wheelControllers = CreateControllers<PID, 4>(speedPID);
//// auto swerveControllers = CreateControllers<Amplifier<1>, 4>();
//
#define DIRECT_SPEED {Motor_Ctrl_Type_e::Speed, Motor_Ctrl_Type_e::Speed}
#define DIRECT_TORQUE {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Torque}
#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}

BM_grouper<2> leftBMGrouper(0x32, 0xF0);
BM1505<2> leftWheelMotor(DIRECT_SPEED, wheelControllers[0], 0x2, leftBMGrouper);
BM1010<2> leftSuspensionMotor(DIRECT_POSITION, suspensionControllers[0], 0x1, leftBMGrouper);
DM4310<2> leftSteerMotor(DIRECT_POSITION, steerControllers[0], 0x109);
//
BM_grouper<1> rightBMGrouper(0x32, 0x0F);
BM1505<1> rightWheelMotor(DIRECT_SPEED, wheelControllers[1], 0x4, rightBMGrouper);
BM1010<1> rightSuspensionMotor(DIRECT_POSITION, suspensionControllers[1], 0x3, rightBMGrouper);
DM4310<1> rightSteerMotor(DIRECT_POSITION, steerControllers[1], 0x10A);
//
//
RadioMaster_Pocket remote;
UARTBuffer<3, 200> uart3Buffer([](uint8_t *data, size_t length) {
    remote.Decode(data, length);
});


void TaskMotorTest() {
    leftWheelMotor.SetTargetSpeed(0);
    leftSuspensionMotor.SetTargetAngle(40);
    leftSteerMotor.SetMITCommand(0, 0, 0, 20.f, 2.0f);

    rightWheelMotor.SetTargetSpeed(0);
    rightSuspensionMotor.SetTargetAngle(40);
    rightSteerMotor.SetMITCommand(0, 0, 0, 20.f, 2.0f);
}

TASK_EXPORT(TaskMotorTest);