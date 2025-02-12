/*******************************************************************************
 * Copyright (c) 2023.
 * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
 * All rights reserved.
 ******************************************************************************/

#include "ProjectConfig.h"

/*****  示例1 *****/
#include "Emm28.h"
#include "LED.h"
#include "BeepMusic.h"
#include "Chassis.h"
#include "RMD_L_40xx_v3.h"
#include "Motor4010.h"
#include "Motor4315.h"
#include "HO3507.h"
#include "Odrive.h"
#include "Manipulator.h"
#include "D28_5_485.h"
#include "RadioMaster_Zorro.h"
#include "FineSerial.h"

/**
 * @brief LED闪烁
 */
void Task1() {
    static int cnt = 0;
    cnt++;
    if(cnt > 1000) {
        cnt = 0;
        LED::Toggle();
    }
}

/*****  示例2 *****/
/**
 * @brief 音乐播放与切换
 */

void Task2() {
    if(GPIO_PIN_RESET == HAL_GPIO_ReadPin(GPIOA, GPIO_PIN_15)) {
        static int index = 1;
        BeepMusic::MusicChannels[0].Play(index++);
        index %= 3;
    }
    BeepMusic::MusicChannels[0].BeepService();
}

/*****  示例3 *****/

/*****  底盘部分  *****/
constexpr PID_Param_t speedPID = {0.3f, 0.005f, 0.33f, 2000, 2000};

auto wheelControllers = CreateControllers<PID, 4>(speedPID);
auto swerveControllers = CreateControllers<Amplifier<1>, 4>();

//构建组成底盘的各个电机
#define TORQUE_2_SPEED {Motor_Ctrl_Type_e::Torque, Motor_Ctrl_Type_e::Speed}
Motor4010<1> CBRMotor(TORQUE_2_SPEED, wheelControllers[0], 0x144);
Motor4010<1> CBLMotor(TORQUE_2_SPEED, wheelControllers[1], 0x143);
Motor4010<1> CFLMotor(TORQUE_2_SPEED, wheelControllers[2], 0x142);
Motor4010<1> CFRMotor(TORQUE_2_SPEED, wheelControllers[3], 0x141);

#define DIRECT_POSITION {Motor_Ctrl_Type_e::Position, Motor_Ctrl_Type_e::Position}
Motor4315<1> SBRMotor(DIRECT_POSITION, swerveControllers[0], 0x04);
Motor4315<1> SBLMotor(DIRECT_POSITION, swerveControllers[1], 0x03);
Motor4315<1> SFLMotor(DIRECT_POSITION, swerveControllers[2], 0x02);
Motor4315<1> SFRMotor(DIRECT_POSITION, swerveControllers[3], 0x01);


// 首先调取底盘类的构建器，然后使用提供的电机添加函数，将上文构建的电机指针传入构建器，最后由构建器返回构建好的底盘类对象
Chassis chassis = Chassis::Build().
                  AddCFLMotor(CFLMotor).
                  AddCFRMotor(CFRMotor).
                  AddCBLMotor(CBLMotor).
                  AddCBRMotor(CBRMotor).
                  AddSFLMotor(SFLMotor).
                  AddSFRMotor(SFRMotor).
                  AddSBLMotor(SBLMotor).
                  AddSBRMotor(SBRMotor).
                  Build();


// constexpr float timeConsumed[22]{
//     2, 3.5, 3.5, 3.5, 6, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 3.5, 6, 4, 3.5, 4, 3.5, 3.5, 4, 6, 2
// };

float pathTask1[3][7] = {
    {-0.4, -0.2, -0.15, 0, 0, 0, 2},
    {-0.8, 0, -0.15, 0, 0, 0, 4},
    {-1.49, 0, -0.15, 0, 0, 0, 4}
};
float pathTask2[4][7] = {
    {-1.29, 0.2, -0.15, 0, 0, 0, 2},
    {-1.12, 0, -0.35, -0.2, 0, 0, 1.5},
    {-1.05, 0, -1.9, 0, 0, 0, 5},
    // {-1.05, 0, -1.9, 0, PI*1.03, 0, 4}
    {-1.05, 0, -1.9, 0, PI*1.03, 0, 4}
};
float pathTask3[3][7] = {
    {-1.75, -0.2, -2.0, 0, PI*1.03, 0, 2},
    {-1.9, 0, -1.8, 0.2, PI*0.53, 0, 2},
    {-1.9, 0, -1.1, 0, PI*0.534, 0, 2},};
float pathTask4[3][7] = {
    {-2, 0, -0.15, 0, PI / 2, 0, 3},
    {-2, 0, -0.15, 0, 0, 0, 3},
    {-1.49, 0, -0.15, 0, 0, 0, 3}, //第二次转盘
};
float pathTask5[4][7] = {
    {-1.29, 0.2, -0.15, 0, 0, 0, 2},
    {-1.12, 0, -0.35, -0.2, 0, 0, 2},
    {-1.1, 0, -1.9, 0, 0, 0, 5},
    {-1.1, 0, -1.9, 0, PI, 0, 4}
};
float pathTask6[3][7] = {
    {-2, 0, -1.9, 0, PI, 0, 3},
    {-2, 0, -1.9, 0, PI / 2, 0, 3},
    {-2, 0, -0.96, 0, PI / 2, 0, 3},
};
float pathTask7[4][7] = {
    {-2, 0, -0.15, 0, PI / 2, 0, 3},
    {-2, 0, -0.15, 0, 0, 0, 3},
    {-0.5, -0.2, -0.15, 0, 0, 0, 3},
    {0, 0, 0, 0, 0, 0, 6},
};
float* path[7] = {&pathTask1[0][0],&pathTask2[0][0],&pathTask3[0][0],&pathTask4[0][0],&pathTask5[0][0],&pathTask6[0][0],&pathTask7[0][0]};



enum class ChassisTask{
    TO_PLATE_1 = 0X00,
    TO_PROCESSING_AREA_1 = 0X01,
    TO_STORAGE_1 = 0X02,
    TO_PLATE_2 = 0X03,
    TO_PROCESSING_AREA_2 = 0X04,
    TO_STORAGE_2 = 0X05,
    BACK = 0X06,
    NONE = 0X07
}chassisTask = ChassisTask::NONE;

RoutePlanning route_planning(0.5);//Kp为误差补偿系数
bool nextPoint = false;
bool isMissionStart = false;
bool isTaskPub = false;
bool isTargetReachedMsgPub = false;
uint8_t pathCnt{0};
uint8_t backForceCounter{0};
uint8_t uploadCnt{0};





void Task3() {
    if(HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_6) == GPIO_PIN_SET && !isMissionStart){
        static uint32_t pinCnt{0};
        pinCnt++;
        if(pinCnt > 100)
        {
            isMissionStart = true;
            FineSerial<5>::GetInstance().AvtivateUpload();
            chassisTask = ChassisTask::TO_PLATE_1;
        }
    }

    if(!isMissionStart){return;}

    uploadCnt = FineSerial<5>::GetInstance().cnt;
    // if(nextPoint)
    // {
    //     nextPoint = false;
    //     switch (pathCnt)
    //     {
    //     case 0:
    //         route_planning.AddTarget(path[pathCnt], 2);
    //         break;
    //     case 1: route_planning.AddTarget(path[pathCnt], 4);
    //         break;
    //     case 2: route_planning.AddTarget(path[pathCnt], 3);
    //         break;
    //     case 3: route_planning.AddTarget(path[pathCnt], 3);
    //         break;
    //     case 4: route_planning.AddTarget(path[pathCnt], 4);
    //         break;
    //     case 5: route_planning.AddTarget(path[pathCnt], 3);
    //         break;
    //     case 6: route_planning.AddTarget(path[pathCnt], 4);
    //         break;
    //     }
    //     pathCnt++;
    // }




    static uint8_t lastBFCounter{0};
    switch (chassisTask)
    {
        case ChassisTask::TO_PLATE_1:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 3);
                isTaskPub = true;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            if(backForceCounter > lastBFCounter && backForceCounter < 3)
            {
                lastBFCounter = backForceCounter;
                FineSerial<5>::GetInstance().AvtivateUpload();
            }
            // backForceCounter = 6;
            if(backForceCounter >= 3)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::TO_PROCESSING_AREA_1;
                backForceCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::TO_PROCESSING_AREA_1:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 4);
                isTaskPub = true;
                backForceCounter = 0;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            // backForceCounter = 6;
            if(backForceCounter >= 6 && route_planning.isFinished)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::TO_STORAGE_1;
                backForceCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::TO_STORAGE_1:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 3);
                isTaskPub = true;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            if(backForceCounter >= 3 && route_planning.isFinished && FineSerial<5>::GetInstance().timeMsgNotReceived > 0.4)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::TO_PLATE_2;
                backForceCounter = 0;
                lastBFCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::TO_PLATE_2:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 3);
                isTaskPub = true;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            if(backForceCounter > lastBFCounter && backForceCounter < 3)
            {
                lastBFCounter = backForceCounter;
                FineSerial<5>::GetInstance().AvtivateUpload();
            }
            if(backForceCounter >= 3 && route_planning.isFinished)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::TO_PROCESSING_AREA_2;
                backForceCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::TO_PROCESSING_AREA_2:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 4);
                isTaskPub = true;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            if(backForceCounter >= 6 && route_planning.isFinished)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::TO_STORAGE_2;
                backForceCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::TO_STORAGE_2:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 3);
                isTaskPub = true;
            }
            if(route_planning.isFinished && !isTargetReachedMsgPub)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTargetReachedMsgPub = true;
            }
            if(backForceCounter >= 3 && route_planning.isFinished && FineSerial<5>::GetInstance().timeMsgNotReceived > 0.4)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                isTaskPub = false;
                chassisTask = ChassisTask::BACK;
                backForceCounter = 0;
                isTargetReachedMsgPub = false;
            }
            break;
        case ChassisTask::BACK:
            if(!isTaskPub)
            {
                route_planning.AddTarget(path[static_cast<uint8_t>(chassisTask)], 4);
                isTaskPub = true;
            }
            if(route_planning.isFinished)
            {
                FineSerial<5>::GetInstance().AvtivateUpload();
                chassisTask = ChassisTask::NONE;
            }
            break;
        case ChassisTask::NONE:break;
        default: break;
    }


    /*****  单次任务部分  *****/
    if (!FineSerial<5>::GetInstance().isCurrentTaskFinished){
        switch (FineSerial<5>::GetInstance().singleCommand)
        {
            case SingleCommandType::NONE:
                break;
            case SingleCommandType::MISSION_START:
                // manipulator.GetInitCommand = true;
                FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
                break;
            case SingleCommandType::SET_PATH_POINT:
                // route_planning.AddTarget(FineSerial<5>::GetInstance().path_point.x, 0,FineSerial<5>::GetInstance().path_point.y, 0,FineSerial<5>::GetInstance().path_point.yaw, 0, timeConsumed[pathCounter++]);
                FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
                break;
            case SingleCommandType::ODOMETRY_OFFSET:
                float offsetX = FineSerial<5>::GetInstance().offset_data.y * cosf(chassis.yaw) + FineSerial<5>::GetInstance().offset_data.x * sinf(chassis.yaw);
                float offsetY = FineSerial<5>::GetInstance().offset_data.y * -sinf(chassis.yaw) + FineSerial<5>::GetInstance().offset_data.x * cosf(chassis.yaw);
                float offsetYaw = FineSerial<5>::GetInstance().offset_data.yaw;
                route_planning.AddTarget(chassis.x, 0, chassis.y, 0, chassis.yaw, 0, 3);
                chassis.OffsetOdometry(-offsetX, -offsetY, -offsetYaw);
                FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
                break;
            // case SingleCommandType::CHASSIS_STOP:
            //     chassis.ChassisSetVelocity(0, 0, 0);
            //     FineSerial<5>::GetInstance().isCurrentTaskFinished = true;
            //     break;
            // case SingleCommandType::END_EFFECTOR:
            //     break;
        }
    }
}



uint8_t manipulatorCommand[28]{};
uint8_t cnt{0};

struct ManipulatorAngle{
    float angleA{0};
    float angleB{0};
    float angleC{0};
    float angleD{0};
    float angleE{0};
    float angleF{0};
    uint8_t endEffecctor{0};
}__packed manipulator_angle;

void ManipulatorBackForceCounter()
{
    static bool isForward = false;
    if(manipulator_angle.angleA < - 2.2)//FORCE
    {
        isForward = true;
    }
    if(manipulator_angle.angleA > - 2 && isForward)//BACK
    {
        backForceCounter++;
        isForward = false;
    }
}

// void Task4(){
//     manipulatorCommand[0] = 0xAA;
//     memcpy(manipulatorCommand + 1, FineSerial<5>::GetInstance().manipulator_angle, 24);
//     manipulatorCommand[25] = static_cast<uint8_t>(FineSerial<5>::GetInstance().endEffectorState);
//     manipulatorCommand[26] = static_cast<uint8_t>(isMissionStart);
//     manipulatorCommand[27] = CRC8Calc(manipulatorCommand + 1, 26);
//     manipulatorCommand[28] = 0xBB;
//
//     cnt++;
//     if (cnt >= 5){
//         UARTBaseLite<4>::GetInstance().Transmit(manipulatorCommand, 29);
//         cnt = 0;
//     }
// }

void Task4(){

    /*****  循环任务部分  *****/
    chassis.ChassisSetVelocity(route_planning.FBVel, route_planning.LRVel, route_planning.RTVel);
    route_planning.Update(chassis.chassisPos[0][0], chassis.WCSVelocity[0][0], chassis.chassisPos[1][0],
                  chassis.WCSVelocity[1][0], chassis.chassisPos[2][0], chassis.WCSVelocity[2][0]);


    cnt++;
    if (cnt >= 4){
        manipulatorCommand[0] = 0xAA;
        memcpy(manipulatorCommand + 1, FineSerial<5>::GetInstance().manipulator_angle, 24);
        manipulatorCommand[25] = static_cast<uint8_t>(FineSerial<5>::GetInstance().endEffectorState);
        manipulatorCommand[26] = CRC8Calc(manipulatorCommand + 1, 25);
        manipulatorCommand[27] = 0xBB;
        UARTBaseLite<4>::GetInstance().Transmit(manipulatorCommand, 28);
        cnt = 0;

        memcpy(&manipulator_angle,manipulatorCommand+1,25);
    }
}





/**
 * @brief 用户初始化
 */

#ifdef __cplusplus
extern "C" {
#endif

void Setup() {
    // std::function<void(uint8_t *, uint16_t)> maniDecodeFunc = [](uint8_t* data, uint16_t length){
    //     remote.Decode(data, length);
    // };
    // UARTBaseLite<4>::GetInstance().Bind(maniDecodeFunc);

    // std::function<void(uint8_t *, uint16_t)> decodeFunc = [](uint8_t* data, uint16_t length){
    //     FineSerial<5>::GetInstance().Decode(data, length);
    // };
    // UARTBaseLite<5>::GetInstance().Bind(decodeFunc);

    RS485_Base<1>::GetInstance().SetDivisionFactor(4);
    RS485_Base<2>::GetInstance().SetDivisionFactor(100);
}

/**
 * @brief 主循环，优先级低于定时器中断，不确定执行频率
 */
void Loop() {

}

#ifdef __cplusplus
}
#endif

/*****  不要修改以下代码 *****/

#ifdef __cplusplus
extern "C" {
#endif


void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
    if(htim == &TIM_Control) {
        HAL_IWDG_Refresh(&hiwdg);
        DeviceBase::DevicesHandle();
        Task1();
        Task2();
        Task3();
        Task4();
        ManipulatorBackForceCounter();

        CAN_Bus<1>::TxLoader();
        CAN_Bus<2>::TxLoader();
    }
}

#ifdef __cplusplus
}
#endif
