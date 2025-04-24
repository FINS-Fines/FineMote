#ifndef RS485DEV_MONITOR_HPP
#define RS485DEV_MONITOR_HPP

#include "RS485Dev.hpp"

// 接收 RS485 消息， PWM 占空比输出
class RS485DevMonitor : public RS485DevBase {
#define MONITOR_ARR_SIZE (10)
public:
  explicit RS485DevMonitor() = default;

  // 获取当前值
  bool GetValue (std::array<float, MONITOR_ARR_SIZE> & values) const {
    if (values_.empty()) {
      return false;
    }

    // 将当前值赋值给传入的数组
    std::copy(values_.begin(), values_.end(), values.begin());
    return true;
  };

  // 按照指定长度将数据拷贝出去
  template<uint16_t CURRENT_MONITOR_ARR_SIZE>
  bool GetValue (std::array<float, CURRENT_MONITOR_ARR_SIZE> & values) const {
    if (CURRENT_MONITOR_ARR_SIZE > MONITOR_ARR_SIZE) {
      return false;
    }

    // 将当前值赋值给传入的数组
    std::copy(values_.begin(), values_.begin() + CURRENT_MONITOR_ARR_SIZE, values.begin());
    return true;
  };

  // 按照指定位置截断数据
  template<uint16_t CURRENT_MONITOR_ARR_BEGIN, uint16_t CURRENT_MONITOR_ARR_SIZE>
  bool GetValue (std::array<float, CURRENT_MONITOR_ARR_SIZE> & values) const {
    if (CURRENT_MONITOR_ARR_BEGIN + CURRENT_MONITOR_ARR_SIZE > MONITOR_ARR_SIZE) {
      return false;
    }

    // 将当前值赋值给传入的数组
    std::copy(values_.begin() + CURRENT_MONITOR_ARR_BEGIN, values_.begin() + CURRENT_MONITOR_ARR_BEGIN + CURRENT_MONITOR_ARR_SIZE, values.begin());
    return true;
  };

public:
  std::array<float, MONITOR_ARR_SIZE> values_;
};

#endif // RS485DEV_MONITOR_HPP