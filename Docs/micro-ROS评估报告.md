# micro-ROS 模块综合评估报告

> 评估日期：2026-07-13
> 评估范围：Services/MicroROS/、Services/Bus/、BSP/*/FineMote_BSP/、ThirdParty/micro-ROS/

---

## 一、任务1：用 true/false 显示是否存在服务 (Service)

### 现状

当前代码**完全没有实现 ROS 2 Service**。`MicroROS_Agent.hpp` 只提供了 `RosPublisher` 和 `RosSubscriber`，没有 `RosServiceServer` 或 `RosServiceClient`。

但底层库**支持 Service**：
- 预编译库配置了 `RMW_UXRCE_MAX_SERVICES=1`、`RMW_UXRCE_MAX_CLIENTS=1`
- 第三方头文件 `rcl/service.h`、`rclc/service.h` 均可用
- `available_ros2_types` 文件中列出了 `std_srvs/srv/SetBool` 等服务类型

### 分析

"用 true/false 显示是否存在服务" 应该指：在 PC 端通过 micro-ROS agent 可以查询 MCU 端提供了哪些服务。有两层含义：

1. **框架层面**：需要封装 `rclc_service_server_init_default` / `rclc_service_client_init_default`，类似现有的 Pub/Sub 模板
2. **可发现性**：micro-ROS 通过 XRCE-DDS 协议与 agent 通信，创建 service server 后 agent 端可通过 `ros2 service list` 发现。可以增加一个自检服务（如 `/FineMote/health`），让 PC 端直接询问 MCU 是否在线

**建议做法**：仿照 `RosPublisher`/`RosSubscriber` 模式，增加 `RosServiceServer<ReqT, ResT>` 模板类，注册到 executor。

---

## 二、任务2：如何实现与 UART 的解耦

### 现状分析

当前 `MicroROS_Transport` 与 UART 的耦合体现在 3 层：

| 层次 | 文件 | 耦合方式 |
|------|------|---------|
| 传输层 | `MicroROS_Transport.hpp:14` | `#include "Bus/UART_Base.hpp"` — 直接依赖 |
| 构造层 | `MicroROS_Transport.hpp:75-86` | 构造函数中硬编码 `UART_Base<MICRO_ROS_UART_ID>::GetInstance()` |
| 数据层 | `MicroROS_Transport.hpp:145` | 成员变量 `UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_` 是 UART 特有类型 |

耦合链为：

```
MicroROS_Transport  ➜  UART_Base<ID>  ➜  BSP_UART<ID>  ➜  HAL_UART_xxx
       ↕ (DMA双缓冲)
   UARTBuffer<ID,N>  ➜  DoubleBuffer<N>
```

### 解耦方案建议

采用**策略模式 / 抽象传输接口**。micro-ROS 自定义传输本质上只需要 4 个函数：

```cpp
struct IMicrorosTransport {
    virtual bool   Open() = 0;
    virtual bool   Close() = 0;
    virtual size_t Write(const uint8_t* buf, size_t len) = 0;
    virtual size_t Read(uint8_t* buf, size_t len, int timeout) = 0;
};
```

`MicroROS_Transport` 不再持有 `UART_Base` 和 `UARTBuffer`，而是持有一个 `IMicrorosTransport*` 指针。具体实现可以是 `UartTransport`、`UsbCdcTransport`、`SpiTransport` 等，通过依赖注入或模板参数传入。这样：

- 换用 USB CDC 不需要改动 `MicroROS_Transport`
- 单元测试可以 mock 传输层
- UART 的 DMA/环形缓冲细节被封装在 `UartTransport` 实现内部

---

## 三、任务3：BSP 接口不统一问题

这是三个任务中最严重的架构问题。存在**两类典型的接口不统一**：

### 3.1 BSP_UART.h — 冗余的 copy-paste（轻度问题）

3 个板子的 `BSP_UART.h` 90% 相同，唯一差异是 **MC_Board_02 多了 SBUS DMA 特化**（第 71-73 行）：

```cpp
// 仅 MC_Board_02 有这段：
template<> inline void BSP_UART<3>::Receive(uint8_t *data, uint16_t size) {
    HAL_UARTEx_ReceiveToIdle_DMA(BSP_UARTList[3], data, 25);
}
```

而 `BSP_UART.cpp` 是 **100% 完全相同**的，在 3 个板子各存一份。

### 3.2 BSP_CAN.h — 接口相同但实现完全不同（严重问题）

`BSP_CAN<ID>::Transmit()` 和 `BSP_CAN<ID>::Receive()` 使用了统一的 `FineMote_CAN_HeaderTypeDef` 作为接口参数——**这点做得对**。但 `PeriphralInit()` 内部完全割裂：

| | MC_Board / Robomaster_C (bxCAN) | MC_Board_02 (FDCAN) |
|---|---|---|
| HAL 句柄类型 | `CAN_HandleTypeDef` | `FDCAN_HandleTypeDef` |
| 过滤器类型 | `CAN_FilterTypeDef` | `FDCAN_FilterTypeDef` |
| 激活通知 | `HAL_CAN_ActivateNotification` | `HAL_FDCAN_ActivateNotification` |
| 启动 | `HAL_CAN_Start` | `HAL_FDCAN_Start` |
| 中断回调 | `HAL_CAN_RxFifo0MsgPendingCallback` | `HAL_FDCAN_RxFifo0Callback` |
| 发送函数 | `HAL_CAN_AddTxMessage` | `HAL_FDCAN_AddMessageToTxFifoQ` |

MC_Board_02 的 `BSP_CAN.h` 里还**缺少** `#include "Bus/CAN_Base.hpp"`（只有 `CAN_Header.hpp`），导致它无法独立参与 `FineMoteAux_CAN` 的编译时匹配。

`BSP_CAN.cpp` 的差异更大——MC_Board_02 完全使用不同的 HAL 回调名（`HAL_FDCAN_*`），且 bxCAN 的 3 个 mailbox 回调全部被注释掉了。

### 3.3 统一建议

提取公共模板到共享目录，板级差异通过**特化 + 编译期条件**处理：

```
BSP/
  Common/
    BSP_UART.h        ← 公共模板（3个板子当前完全相同）
    BSP_CAN.h         ← 公共模板 + PeriphralInit 用 Policy 分离
  MC_Board/
    FineMote_BSP/     ← 只保留板级特化和 Board.h
  MC_Board_02/
    FineMote_BSP/
  Robomaster_C/
    FineMote_BSP/
```

对于 CAN，把 `PeriphralInit()` 作为一个独立策略类，在 `Board.h` 中通过类型别名指定：
- `using CAN_Policy = BxCAN_Policy;` (MC_Board, Robomaster_C)
- `using CAN_Policy = FdCAN_Policy;` (MC_Board_02)

对于 MC_Board_02 中 SBUS UART 的 DMA 特化场景，用一个**可选的特化钩子**（如 `BSP_UART_Receive_Impl<ID>`）在板级头文件中提供，公共模板在默认情况下回退到中断模式。

---

## 四、发现的其他 micro-ROS 问题

### 4.1 `MicroROS_Transport::Read()` 存在代码异味（中等）

```cpp
static size_t Read(..., int timeout, ...) {
    auto& self = GetInstance();
    int remain = timeout;
    do {
        size_t n = self.ReadRxData(buf, len);
        if (n > 0) return n;
        if (timeout <= 0) break;   // ← 这里检查的是 timeout，不是 remain
        usleep(5000);
        remain -= 5;
    } while (remain > 0);
    return 0;
}
```

逻辑上恰好能工作（因为 `timeout` 不变，`timeout <= 0` 只在首次进入时触发 break，后续靠 `remain > 0` 控制），但 `timeout <= 0` 应该是 `remain <= 0`。当前写法让读者误以为这是 bug。

### 4.2 `Write()` 重试循环阻塞 100ms（中等）

```cpp
for (int i = 0; i < 100; ++i) {
    if (self.TryTransmit(buf, len)) return len;
    usleep(1000);
}
```

在 micro-ROS 线程中忙等最多 100ms，期间 executor 无法 spin。对于实时控制场景，这意味着 100ms 内无法处理任何 sub 回调或 pub 任务。应改用 FreeRTOS 信号量/任务通知，或者用队列异步发送后立即返回。

### 4.3 ERROR 恢复后 Agent 重新初始化可能失败（严重）

状态机路径：`ERROR → sleep(1) → WAITING_AGENT → INITIALIZING → RUNNING`

`Cleanup()` 调用了 `agent->Fini()`，Fini 内部调用 `rcl_publisher_fini` / `rcl_subscription_fini`。但 Fini 之后，publisher/subscriber 的 `rcl_*_t` 结构体**没有重新置零**（应调用 `= rcl_get_zero_initialized_publisher()` 等）。再次进入 INITIALIZING 时，`agent->Init()` 调用 `rclc_publisher_init_default(&publisher_, ...)`，如果底层 rcl 没有对传入的非零初始化结构体做防御性处理，可能导致内存泄漏或初始化失败。

**修复方向**：在 `Fini()` 末尾重新赋值为零初始化状态，或在 `Init()` 开头做零初始化。

### 4.4 缺少线程安全保护（中等）

- `MicroROS_Manager::Handle()` 在独立线程中运行，但 `agents_` 列表没有互斥锁保护
- `RegisterAgent()` 通常在静态初始化阶段调用，但如果运行时动态注册会产生 data race
- `state_` 枚举的读写也没有原子保护

### 4.5 `ChassisBase.hpp` 引入了对 micro-ROS 的头文件强依赖（轻度）

```cpp
#include "MicroROS/MicroROS_Agent.hpp"
#include <geometry_msgs/msg/twist.h>
```

即使 `WITH_MICRO_ROS = false`，所有包含 `ChassisBase.hpp` 的编译单元都需要能解析 micro-ROS 头文件路径。`if constexpr` 只能消除运行时代码，不能消除编译期依赖。建议将 `GetRosBinder()` 和 `UpdateToRos()` 移到单独的 mixin 或 traits 中，在 `WITH_MICRO_ROS = false` 时完全不实例化。

### 4.6 缺少链路质量监控能力（建议）

当前只在 `HandleRunning()` 中每 1 秒 ping agent 来检测断开，但没有暴露任何诊断信息：
- 收/发字节数统计
- 重连次数
- 最后一次成功通信的时间戳
- 环形缓冲区溢出次数

这些对于生产环境调试非常有用，建议至少暴露几个 `std::atomic<uint32_t>` 计数器。

### 4.7 micro-ROS 线程栈大小可能偏紧（建议关注）

```cpp
constexpr size_t STACK_SIZE = 20 * 1024;
```

20KB 对于 micro-ROS + XRCE-DDS + FreeRTOS-POSIX 来说是偏紧的。如果将来增加 Service 支持或更多 topic，栈溢出风险会增大。建议用 `uxTaskGetStackHighWaterMark` 监控实际使用量。

---

## 总结

| 优先级 | 问题 | 类别 |
|--------|------|------|
| **P0** | BSP_CAN.h / BSP_UART.h 三份 copy-paste 需统一 | 任务3 |
| **P0** | ERROR 恢复后 Agent 未重新零初始化 | 新发现 |
| **P1** | MicroROS_Transport 与 UART 硬耦合 | 任务2 |
| **P1** | `Read()` 中变量名混淆 (`timeout` vs `remain`) | 新发现 |
| **P1** | `Write()` 中 100ms 忙等阻塞 executor | 新发现 |
| **P2** | 缺少 ROS 2 Service 支持 | 任务1 |
| **P2** | 缺少线程安全保护 | 新发现 |
| **P2** | ChassisBase.hpp 强依赖 micro-ROS 头文件 | 新发现 |
| **P3** | 缺少链路质量监控 | 新发现 |
| **P3** | 线程栈大小需验证 | 新发现 |
