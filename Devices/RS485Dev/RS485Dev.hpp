#ifndef RS485DEV_HPP
#define RS485DEV_HPP

#include "Bus/RS485_Base.h"

// 接收 RS485 消息， PWM 占空比输出
class RS485DevBase : public DeviceBase {
public:
  // 构造将设备挂在总线
  // @param addr 设备地址
  // @param div 分频系数
  explicit RS485DevBase() = default;

protected:
  virtual void Require() {}

  virtual bool Check() {return false;}

  virtual void Update() {}

  void Handle() override {
    if (Check())
      Update();
    Require();
  }
};

#endif // RS485DEV_HPP