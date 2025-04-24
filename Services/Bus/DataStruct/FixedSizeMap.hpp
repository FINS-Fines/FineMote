#ifndef FIXED_SIZE_MAP_HPP
#define FIXED_SIZE_MAP_HPP

#include <array>

template <typename Key, typename Value, size_t Size>
class FixedSizeMap {
private:
  std::pair<Key, Value> data[Size];
  size_t size_ = 0;

public:
  // 插入元素
  bool insert(const Key& key, const Value& value) {
    if (size_ >= Size) {
      return false; // 容量已满
    }

    // 遍历 key 是否已存在
    for (size_t i = 0; i < size_; ++i) {
      if (data[i].first == key) {
        data[i].second = value; // 更新值
        return true;
      }
    }
    // 插入新元素
    data[size_++] = std::make_pair(key, value);
    return true;
  }

  // 查找元素
  bool fetch(const Key& key, Value& value) const {
    for (size_t i = 0; i < size_; ++i) {
      if (data[i].first == key) {
        value = data[i].second;
        return true; // 找到元素
      }
    }
    return false; // 未找到元素
  }

  // 获取元素数量
  size_t size() const {
    return size_;
  }

  // 判断是否为空
  bool empty() const {
    return size_ == 0;
  }
};

#endif