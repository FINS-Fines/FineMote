#ifndef DOUBLE_BUFF_HPP
#define DOUBLE_BUFF_HPP

#include <array>

template<typename T, size_t N>
class DoubleBuffer {
public:
  DoubleBuffer() : isWriting(true) {}

  // 写入数据到当前写入缓冲区
  void write(const std::array<T, N>& data) {
    size_t writeIndex = isWriting ? 0 : 1;
    auto& writeBuffer = buffers[writeIndex];
    writeBuffer = data;
    isWriting = !isWriting;
  }

  // 读取数据，返回当前读取缓冲区的数据
  std::array<T, N> read() {
    size_t readIndex = isWriting ? 1 : 0;
    return buffers[readIndex];
  }

private:
  std::array<std::array<T, N>, 2> buffers;
  bool isWriting;
};

#endif