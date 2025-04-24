//
// Created by jenny on 25-3-17.
//

#include "AirCooler.h"

namespace FuelCell {

AirCoolerBuilder AirCooler::Build() {
  return {};
}

void AirCooler::CurrPowerCalc() {
  power_tgt = 0.0f;
}

void AirCooler::CurrTempsCalc() {
  // tgt
  TemperatureConf::GetConfig().fetch(power_tgt, temps_tgt);

  // curr
  temps_.GetValue<0, 4>(temps_inlet_);
  temps_.GetValue<4, 4>(temps_outlet_);
};

void AirCooler::FanCompCalc() {
  // PID 控制器计算风扇转速
  comps_inlet_[0] = ctrl_1_.Calc();
  comps_inlet_[1] = ctrl_2_.Calc();
  comps_inlet_[2] = ctrl_3_.Calc();
  comps_inlet_[3] = ctrl_4_.Calc();
  comps_outlet_[0] = ctrl_5_.Calc();
  comps_outlet_[1] = ctrl_6_.Calc();
  comps_outlet_[2] = ctrl_7_.Calc();
  comps_outlet_[3] = ctrl_8_.Calc();

  // 设置风扇占空比
  fan_1_.SetComp(comps_inlet_[0]);
  fan_2_.SetComp(comps_inlet_[1]);
  fan_3_.SetComp(comps_inlet_[2]);
  fan_4_.SetComp(comps_inlet_[3]);
  fan_5_.SetComp(comps_outlet_[0]);
  fan_6_.SetComp(comps_outlet_[1]);
  fan_7_.SetComp(comps_outlet_[2]);
  fan_8_.SetComp(comps_outlet_[3]);
}

}