# micro-ROS Service 层讲解

> 基于 FineMote `feat/micro-ROS` 分支，2026-07-14 对话整理。

---

## 1. 背景：ROS 2 与 micro-ROS 的关系

**ROS 2** 是机器人领域主流的通信框架，提供发布/订阅（Publish/Subscribe）机制让各模块通过"话题（Topic）"通信。但 ROS 2 需要 Linux + 较强 CPU，STM32 单片机跑不了。

**micro-ROS** 把 ROS 2 的通信协议栈裁剪后搬到 MCU 上。MCU 固件跑 micro-ROS 后即可与 PC 上的 ROS 2 系统直接通信。

### 通信拓扑

```
┌─────────────────────────────────┐
│         PC (Linux)              │
│  ROS 2 + micro_ros_agent        │
│  （串口守护进程，桥接 MCU 与 ROS）│
└────────────┬────────────────────┘
             │ UART (921600 bps)
┌────────────┴────────────────────┐
│       STM32H7 单片机            │
│   micro-ROS Client              │
│   Publisher（发）/ Subscriber（收）│
└─────────────────────────────────┘
```

核心约束：**MCU 永远是 Client，PC 上必须跑 Agent 做中转**，两个 MCU 不能直接通信。

---

## 2. Service 层四个文件

```
Services/MicroROS/
├── MicroROS_Transport.hpp    # 物理传输层（UART + DMA + 环形缓冲）
├── MicroROS_MessageTypes.hpp # 消息类型注册（C++ 类型 ↔ ROS 消息类型）
├── MicroROS_Agent.hpp        # Publisher / Subscriber 模板类
└── MicroROS_Manager.hpp      # 生命周期管理器（状态机 + FreeRTOS 线程）
```

### 2.1 MicroROS_Transport.hpp — 物理传输层

向 micro-ROS 提供 4 个函数：`open`、`close`、`write`、`read`。

- **RX**：2048 字节环形缓冲区，读写指针用 `std::atomic` 无锁操作（DMA 中断和 ROS 线程并发安全）
- **TX**：CAS 发送锁（`tx_busy_`），忙则重试最多 100 次（100ms 超时）
- 完全不依赖 FreeRTOS，仅用 C++ 标准库

### 2.2 MicroROS_MessageTypes.hpp — 消息类型注册

用宏将 C++ 结构体与 ROS 消息类型绑定：

```cpp
DEFINE_MICROROS_MSG(sensor_msgs__msg__JointState, sensor_msgs, msg, JointState)
//                   ↑ C++ 类型                         ↑ 包名   ↑ 子目录 ↑ 消息名
```

展开后生成 `RosMsgTraits<T>` 特化，提供：
- `registered = true`（编译期检查，防未注册类型）
- `GetTypeSupport()`（序列化/反序列化配方）
- `name = "JointState"`（用于拼接 topic 名）

已注册消息：`JointState`、`Twist`、`Bool`、`Int32`。

### 2.3 MicroROS_Manager.hpp — 生命周期管理器

模板单例，跑独立 FreeRTOS 线程（通过 `pthread_create`），四状态机：

```
WAITING_AGENT → INITIALIZING → RUNNING
      ↑                            │
      └───────── ERROR ←───────────┘
```

| 状态 | 行为 |
|------|------|
| WAITING_AGENT | ping Agent（500ms 超时），成功则进入下一状态 |
| INITIALIZING | 初始化 rclc_support → rcl_node → rclc_executor → 逐个 Init Agent |
| RUNNING | spin_some(10ms) → 各 Agent Execute() → 每 5 循环 ping 一次 |
| ERROR | sleep 1s → Cleanup 所有资源 → 回到 WAITING_AGENT |

关键参数：循环 200ms，线程栈 20KB，最大 10 个 Agent，执行器最大 10 个句柄。

### 2.4 MicroROS_Agent.hpp — Publisher / Subscriber 模板类

文件包含（按代码顺序）：

| 组件 | 作用 |
|------|------|
| `has_GetRosBinder<T>` trait | 编译期探测类型是否有 `GetRosBinder()` 方法 |
| `callback_message_type<T>` trait | 编译期提取函数/lambda 的参数类型 |
| `ROSAgent` 基类 | 统一生命周期接口（Init / Execute / Fini），构造时自动注册到 Manager |
| `RosPublisher<MsgT>` | 发布者，Reliable QoS，topic 格式 `FineMote/{obj}/{MsgName}` |
| `RosSubscriber<MsgT>` | 订阅者，Best-Effort QoS，topic 格式 `FineMote/{name}` |
| CTAD 推导指南（3 个） | 让编译器自动推导模板参数，无需手动写 `<MsgT>` |

---

## 3. converter_ 与 GetRosBinder() 机制

### 3.1 概念

- `GetRosBinder()`：业务对象（电机、底盘等）提供的方法，返回一个 lambda，签名 `void(MsgT&)`
- `converter_`：`std::function<void(MsgT&)>`，把上述 lambda 存起来
- 每个周期 `Execute()` 调用 `converter_(msg_)` 填充最新数据，然后 `rcl_publish()` 发送

### 3.2 实际例子

**底盘**（`ChassisBase.hpp`）：

```cpp
auto GetRosBinder() {
    return [this](geometry_msgs__msg__Twist& msg) {
        this->UpdateToRos(msg);
    };
}
// UpdateToRos 内从 StateSnapshot 读最新速度，填入 msg.linear / msg.angular
```

**电机**（`MotorBase.hpp`）：

```cpp
auto GetRosBinder() {
    return [this](sensor_msgs__msg__JointState& msg) {
        this->UpdateToRos(msg);
    };
}
// UpdateToRos 内从 StateSnapshot 读 position / velocity / effort，填入 msg
```

**直接 lambda**（`TaskPOVChassis.cpp`）：

```cpp
RosPublisher pub_hb("heartbeat", [](std_msgs__msg__Int32& msg) {
    msg.data = count++;
});
```

### 3.3 完整数据流（以电机为例）

```
CAN 中断                      每 200ms 周期
    │                              │
    ▼                              ▼
CommitState(newState)         Execute()
    │                              │
    ▼                              ▼
stateSnapshot_.Commit()      converter_(msg_)
  (写双缓冲写端)                   │
                                  ▼
                            motor.UpdateToRos(msg)
                                  │
                                  ▼
                            stateSnapshot_.Read()
                              (读双缓冲读端)
                                  │
                                  ▼
                            msg.position = s.position
                            msg.velocity = s.speed
                            msg.effort   = s.torque
                                  │
                                  ▼
                            rcl_publish(&msg)
                              (UART → PC Agent → ROS 2)
```

### 3.4 设计意图

依赖注入 / 策略模式：`RosPublisher` 不需要知道数据从哪来，只要 `converter_` 能填满 `msg_` 即可。有 `GetRosBinder()` 的对象用重载 1，其他场景用重载 2（直接传 lambda），殊途同归。

---

## 4. 关键设计决策

| 决策 | 说明 |
|------|------|
| Publisher 用 Reliable QoS | 关键状态消息保证送达 |
| Subscriber 用 Best-Effort QoS | 高频指令优先吞吐，丢一帧无所谓 |
| 消息存在 Publisher 内部 | 生命周期安全，不会悬空指针 |
| 构造时自动注册 Agent | 不需手动调 RegisterAgent() |
| CTAD 推导模板参数 | 不用手动写 `RosPublisher<JointState>` |
| Transport 不依赖 FreeRTOS | 用 `std::atomic` 替代 RTOS 原语 |
| 编译期 `static_assert` 检查 | 未注册的消息类型在编译时就报错 |
