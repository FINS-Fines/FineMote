// /*******************************************************************************
// * Copyright (c) 2024.
//  * IWIN-FINS Lab, Shanghai Jiao Tong University, Shanghai, China.
//  * All rights reserved.
//  ******************************************************************************/

#ifndef AIR_COOLER_H
#define AIR_COOLER_H

#include "ControlBase.h"
#include "DeviceBase.h"
#include "Matrix/matrix.h"
#include "ProjectConfig.h"
#include "RS485Dev_PWMFan.hpp"
#include "RS485Dev_TempMonitor_8p.hpp"
#include "FixedSizeMap.hpp"

namespace FuelCell {

/**
 * 12(fan) 34(fan) 56(fan) 78(fan)
 * 0,1,2,3 in 4,5,6,7 out
 */
class AirCoolerBuilder;
class TemperatureConf;

class AirCooler : public DeviceBase {
#define AIR_COOLER_DIVISION (1)
public:
    friend class AirCoolerBuilder;
    AirCooler(
    RS485DevActor* fan_1, ControllerBase* ctrl_1,
    RS485DevActor* fan_2, ControllerBase* ctrl_2,
    RS485DevActor* fan_3, ControllerBase* ctrl_3,
    RS485DevActor* fan_4, ControllerBase* ctrl_4,
    RS485DevActor* fan_5, ControllerBase* ctrl_5,
    RS485DevActor* fan_6, ControllerBase* ctrl_6,
    RS485DevActor* fan_7, ControllerBase* ctrl_7,
    RS485DevActor* fan_8, ControllerBase* ctrl_8,
    RS485DevMonitor* temps,
    uint32_t div = AIR_COOLER_DIVISION):
    fan_1_(*fan_1), ctrl_1_(*ctrl_1),
    fan_2_(*fan_2), ctrl_2_(*ctrl_2),
    fan_3_(*fan_3), ctrl_3_(*ctrl_3),
    fan_4_(*fan_4), ctrl_4_(*ctrl_4),
    fan_5_(*fan_5), ctrl_5_(*ctrl_5),
    fan_6_(*fan_6), ctrl_6_(*ctrl_6),
    fan_7_(*fan_7), ctrl_7_(*ctrl_7),
    fan_8_(*fan_8), ctrl_8_(*ctrl_8),
    temps_(*temps) {

        this->SetDivisionFactor(div);

        ctrl_1->SetTarget(&temps_tgt); ctrl_1->SetFeedback({&temps_inlet_[0]});
        ctrl_2->SetTarget(&temps_tgt); ctrl_2->SetFeedback({&temps_inlet_[1]});
        ctrl_3->SetTarget(&temps_tgt); ctrl_3->SetFeedback({&temps_inlet_[2]});
        ctrl_4->SetTarget(&temps_tgt); ctrl_4->SetFeedback({&temps_inlet_[3]});
        ctrl_5->SetTarget(&temps_tgt); ctrl_5->SetFeedback({&temps_inlet_[4]});
        ctrl_6->SetTarget(&temps_tgt); ctrl_6->SetFeedback({&temps_inlet_[5]});
        ctrl_7->SetTarget(&temps_tgt); ctrl_7->SetFeedback({&temps_inlet_[6]});
        ctrl_8->SetTarget(&temps_tgt); ctrl_8->SetFeedback({&temps_inlet_[7]});
    }

    void CurrTempsCalc();

    void CurrPowerCalc();

    void FanCompCalc();

    void Handle() override {
        CurrPowerCalc();
        CurrTempsCalc();
        FanCompCalc();
    }

public:
    std::array<float, 4> temps_inlet_{};
    std::array<float, 4> temps_outlet_{};
    std::array<float, 4> comps_inlet_{};
    std::array<float, 4> comps_outlet_{};

    float temps_tgt{};
    float power_tgt{};

    RS485DevActor &fan_1_, &fan_2_, &fan_3_, &fan_4_;
    RS485DevActor &fan_5_, &fan_6_, &fan_7_, &fan_8_;
    ControllerBase &ctrl_1_, &ctrl_2_, &ctrl_3_, &ctrl_4_;
    ControllerBase &ctrl_5_, &ctrl_6_, &ctrl_7_, &ctrl_8_;

    RS485DevMonitor &temps_;

    static AirCoolerBuilder Build();
};

class AirCoolerBuilder {
private:
    RS485DevActor* fan_1_p{nullptr};
    RS485DevActor* fan_2_p{nullptr};
    RS485DevActor* fan_3_p{nullptr};
    RS485DevActor* fan_4_p{nullptr};
    RS485DevActor* fan_5_p{nullptr};
    RS485DevActor* fan_6_p{nullptr};
    RS485DevActor* fan_7_p{nullptr};
    RS485DevActor* fan_8_p{nullptr};

    ControllerBase* ctrl_1{nullptr};
    ControllerBase* ctrl_2{nullptr};
    ControllerBase* ctrl_3{nullptr};
    ControllerBase* ctrl_4{nullptr};
    ControllerBase* ctrl_5{nullptr};
    ControllerBase* ctrl_6{nullptr};
    ControllerBase* ctrl_7{nullptr};
    ControllerBase* ctrl_8{nullptr};

    RS485DevMonitor* temps_p{nullptr};

public:
    AirCoolerBuilder& AddFan0(RS485DevActor* fan, ControllerBase* ctrl) {fan_1_p = fan; ctrl_1 = ctrl; return *this;}
    AirCoolerBuilder& AddFan1(RS485DevActor* fan, ControllerBase* ctrl) {fan_2_p = fan; ctrl_2 = ctrl; return *this;}
    AirCoolerBuilder& AddFan2(RS485DevActor* fan, ControllerBase* ctrl) {fan_3_p = fan; ctrl_3 = ctrl; return *this;}
    AirCoolerBuilder& AddFan3(RS485DevActor* fan, ControllerBase* ctrl) {fan_4_p = fan; ctrl_4 = ctrl; return *this;}
    AirCoolerBuilder& AddFan4(RS485DevActor* fan, ControllerBase* ctrl) {fan_5_p = fan; ctrl_5 = ctrl; return *this;}
    AirCoolerBuilder& AddFan5(RS485DevActor* fan, ControllerBase* ctrl) {fan_6_p = fan; ctrl_6 = ctrl; return *this;}
    AirCoolerBuilder& AddFan6(RS485DevActor* fan, ControllerBase* ctrl) {fan_7_p = fan; ctrl_7 = ctrl; return *this;}
    AirCoolerBuilder& AddFan7(RS485DevActor* fan, ControllerBase* ctrl) {fan_8_p = fan; ctrl_8 = ctrl; return *this;}
    AirCoolerBuilder& AddTempMonitor(RS485DevMonitor* monitor) {temps_p = monitor; return *this;}

    AirCooler Build() {
        if (!(fan_1_p && ctrl_1
            && fan_2_p && ctrl_2
            && fan_3_p && ctrl_3
            && fan_4_p && ctrl_4
            && fan_5_p && ctrl_5
            && fan_6_p && ctrl_6
            && fan_7_p && ctrl_7
            && fan_8_p && ctrl_8
            && temps_p)) {
            Error_Handler();
        }
        return AirCooler{
            fan_1_p, ctrl_1,
            fan_2_p, ctrl_2,
            fan_3_p, ctrl_3,
            fan_4_p, ctrl_4,
            fan_5_p, ctrl_5,
            fan_6_p, ctrl_6,
            fan_7_p, ctrl_7,
            fan_8_p, ctrl_8,
            temps_p};
    }
};

class TemperatureConf {
public:
    static TemperatureConf& Instance() {
        static TemperatureConf instance_;
        return instance_;
    }

    /// @brief 计算功率->温度映射
    static FixedSizeMap<float, float, 5> GetConfig() {
        static FixedSizeMap<float, float, 5> config;
        if (config.empty()) {
            // config.insert({0, 55});
            // config.insert({200, 55});
            // config.insert({400, 55});
            // config.insert({600, 55});
            // config.insert({800, 55});
            config.insert(0, 45);
            config.insert(200, 50);
            config.insert(400, 50);
            config.insert(600, 50);
            config.insert(800, 50);
        }
        return config;
    };

private:
    TemperatureConf() = default;
    ~TemperatureConf() = default;
    TemperatureConf(const TemperatureConf&) = delete;
    TemperatureConf& operator=(const TemperatureConf&) = delete;
};

};

#endif
