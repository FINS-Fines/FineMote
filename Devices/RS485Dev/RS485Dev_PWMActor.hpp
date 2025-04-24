#ifndef RS485DEV_ACTOR_HPP
#define RS485DEV_ACTOR_HPP

#include "RS485Dev.hpp"

// 接收 RS485 消息， PWM 占空比输出
class RS485DevActor : public RS485DevBase {
public:
  explicit RS485DevActor() = default;

  // 设置占空比, 0-100
  void SetComp (const uint16_t compare) {
    if (compare < 0) {
      this->curr_comp_ = 0;
      return;
    }

    if (compare > 100) {
      this->curr_comp_ = 100;
      return;
    }

    this->curr_comp_ = compare * 10;
  };

  // todo: 设置频率, 0-1000
  void SetFreq (const uint16_t freq) {
    this->curr_freq_ = freq;
  };

public:
  uint16_t curr_comp_ = 0; // 当前占空比 0-1000
  uint16_t curr_freq_ = 0; // 当前频率 0-1000

};

#endif // RS485DEV_ACTOR_HPP