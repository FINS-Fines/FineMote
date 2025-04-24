#ifndef FIX_SIZE_QUEUE
#define FIX_SIZE_QUEUE


#include <array>

template<typename T, int N>
class FixedSizeQueue {
private:
  std::array<T, N> queue{};
  size_t front = 0;
  size_t rear = 0;
  size_t count = 0;

public:
  void enqueue(const T& item) {
    if (count == N) {
      // 队列已满，移除队首元素
      front = (front + 1) % N;
      --count;
    }
    queue[rear] = item;
    rear = (rear + 1) % N;
    ++count;
  }

  bool dequeue(T& item) {
    if (isEmpty()) {
      return false;
    }
    item = queue[front];
    front = (front + 1) % N;
    --count;
    return true;
  }

  bool dequeue() {
    if (isEmpty()) {
      return false;
    }
    front = (front + 1) % N;
    --count;
    return true;
  }

  T frontElement() const {
    // if (isEmpty()) {
    //   throw std::out_of_range("Queue is empty");
    // }
    return queue[front];
  }

  T rearElement() const {
    // if (isEmpty()) {
    //   throw std::out_of_range("Queue is empty");
    // }
    return queue[(rear + N - 1) % N];
  }

  bool isEmpty() const {
    return count == 0;
  }

  size_t size() const {
    return count;
  }
};

#endif