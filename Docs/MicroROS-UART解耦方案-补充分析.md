# MicroROS 与 UART 解耦 — 补充分析

> 日期：2026-07-15
> 基于 Docs/MicroROS-UART解耦方案.md 的评审与改进建议
> 范围：Services/MicroROS/MicroROS_Transport.hpp、MicroROS_Manager.hpp

---

## 一、总体判断：可以解耦，值得做

当前 `MicroROS_Transport.hpp` 与 UART 有三层耦合：

| 层 | 位置 | 问题 |
|---|---|---|
| **头文件** | `:14` | `#include "Bus/UART_Base.hpp"` 把整个 Bus 层拖进 micro-ROS |
| **构造行为** | `:81-85` | 构造函数直接调 `UART_Base<ID>::GetInstance().BindTxHandle(...)` |
| **类型成员** | `:145` | `UARTBuffer<ID, 512> dma_buffer_` 是模板类型，换传输方式就得换这个成员 |

耦合链：
```
MicroROS_Transport → UART_Base<ID> → BSP_UART<ID> → HAL_UART
```

---

## 二、对已有方案的评审（Docs/MicroROS-UART解耦方案.md）

已有方案采用**策略模式 + 抽象 Backend 接口**，大方向正确。以下 4 个点值得关注：

### 2.1 虚函数 vs 模板 —— 运行时多态还是编译期多态？

已有方案用纯虚函数接口。每次 `Send()` 走 vtable 间接调用，在 Cortex-M7 上约 2-3 条指令开销。200ms 周期内只有几个 Publisher，可忽略。

若对性能极敏感，可改为**模板参数注入**（编译期多态）：

```cpp
template <typename BackendT>
class MicroROS_Transport { ... };
```

建议先用虚函数方案（更直观），后续如需消除 vtable 再改为模板。两种方式切换成本不大。

### 2.2 Backend 接口设计粒度

已有方案把**数据操作**（`Send`）和**回调注册**（`OnTxComplete`、`OnRxData`）混在同一个接口里。带来的问题是：Backend 构造函数中 DMA 已启动，但 rx_callback 在 `SetBackend()` 时才注册。

初始化时序：

```
Backend 构造（硬件就绪，DoubleBuffer 开始缓冲 DMA 数据）
    → SetBackend()（注册回调，数据开始流入 Transport 环形缓冲）
        → Manager 线程启动（WAITING_AGENT，还没 spin）
```

时序是正确的，但有一个隐含假设：**DoubleBuffer 容量足以容纳 SetBackend 调用前到达的数据**。建议验证。

### 2.3 `tx_busy_` + CAS 锁的归属

已有方案将 `tx_busy_` 留在 Transport 层，Backend 的 `Send()` 不关心并发。这是正确的——**串行化是协议层的职责，不应推给硬件后端**。

### 2.4 `Open()` / `Close()` 是否必须？

当前 UART 实现中二者几乎是空操作，但对 USB CDC / SPI 可能有实际意义（打开/关闭端点、CS 引脚控制等）。保留在接口中，给默认实现 `return true` 即可。

---

## 三、推荐方案

### 3.1 Backend 接口定义（嵌套在 MicroROS_Transport 内部）

```cpp
// MicroROS_Transport.hpp
class MicroROS_Transport {
public:
    struct Backend {
        virtual ~Backend() = default;
        virtual bool Open()  { return true; }
        virtual bool Close() { return true; }
        virtual bool Send(const uint8_t* data, size_t len) = 0;
        virtual void OnTxComplete(std::function<void()>) = 0;
        virtual void OnRxData(std::function<void(const uint8_t*, size_t)>) = 0;
    };

    static MicroROS_Transport& GetInstance();
    void SetBackend(Backend& backend);
    // 4 个 static 函数（Open/Close/Write/Read）不变
};
```

放在 Transport 内部而非独立文件，因为它是 micro-ROS 自定义传输的 1:1 映射，不属于通用抽象。

### 3.2 MicroROS_UartBackend（新建文件）

将当前 Transport 中的 UART 逻辑整体搬迁至此：

```cpp
// Services/MicroROS/MicroROS_UartBackend.hpp
// 持有 UARTBuffer + UART_Base 调用，实现 Backend 接口

class MicroROS_UartBackend : public MicroROS_Transport::Backend {
public:
    MicroROS_UartBackend()
        : dma_buffer_([this](uint8_t* data, size_t size) {
              if (rx_callback_) rx_callback_(data, size);
          })
    {
        UART_Base<MICRO_ROS_UART_ID>::GetInstance().BindTxHandle([this]() {
            if (tx_callback_) tx_callback_();
            return true;
        });
    }

    bool Send(const uint8_t* data, size_t len) override {
        UART_Base<MICRO_ROS_UART_ID>::GetInstance().Transmit(
            const_cast<uint8_t*>(data), static_cast<uint16_t>(len));
        return true;
    }

    void OnTxComplete(std::function<void()> cb) override { tx_callback_ = std::move(cb); }
    void OnRxData(std::function<void(const uint8_t*, size_t)> cb) override { rx_callback_ = std::move(cb); }

private:
    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;
    std::function<void()> tx_callback_;
    std::function<void(const uint8_t*, size_t)> rx_callback_;
};
```

### 3.3 Manager 注入

```cpp
// MicroROS_Manager.hpp 构造函数中
MicroROS_Manager()
{
    static MicroROS_UartBackend uart_backend;
    MicroROS_Transport::GetInstance().SetBackend(uart_backend);
    rmw_uros_set_custom_transport(true, nullptr,
                                  MicroROS_Transport::Open,
                                  MicroROS_Transport::Close,
                                  MicroROS_Transport::Write,
                                  MicroROS_Transport::Read);
    allocator_ = rcl_get_default_allocator();
    StartThread();
}
```

`static` 确保 Backend 生命周期覆盖整个程序运行期（嵌入式无 main 返回，安全）。

---

## 四、Transport 层改造要点

| 操作 | 说明 |
|------|------|
| **移除** `#include "Bus/UART_Base.hpp"` | 头文件依赖解耦 |
| **移除** 成员 `dma_buffer_` | 类型耦合解耦 |
| **移除** 构造函数中 `BindTxHandle` 调用 | 行为耦合解耦 |
| **新增** 成员 `Backend* backend_` + `SetBackend()` | 依赖注入入口 |
| `TryTransmit()` → `backend_->Send()` | 委托给抽象接口 |
| `Open()` / `Close()` → 代理到 backend | 生命周期委托 |
| `SetBackend()` 中注册 `OnTxComplete` / `OnRxData` | 回调桥接 |

**保留在 Transport 中不动**：环形缓冲区（`rx_buf_`、`rx_w_`、`rx_r_`）、`tx_busy_` CAS 锁、`tx_buffer_`、`PushRxData()`、`ReadRxData()`、`ResetRx()`。

---

## 五、文件变更清单

| 文件 | 动作 | 说明 |
|------|------|------|
| `Services/MicroROS/MicroROS_Transport.hpp` | **修改** | 移除 UART 依赖，新增 Backend 接口 + SetBackend |
| `Services/MicroROS/MicroROS_UartBackend.hpp` | **新建** | UART 具体实现 |
| `Services/MicroROS/MicroROS_Manager.hpp` | **修改** | 构造函数中创建 UartBackend 并注入 |
| `BSP/*/Board.h` | **不改** | `MICRO_ROS_UART_ID` 仍定义在各板卡配置中 |
| `Services/Bus/` | **不改** | 仅被 UartBackend 引用，不再被 Transport 引用 |

---

## 六、替换为新传输方式（示例）

```
Services/MicroROS/
├── MicroROS_Transport.hpp         ← 不改
├── MicroROS_Manager.hpp          ← 改一行
├── MicroROS_UartBackend.hpp      ← 保留
├── MicroROS_UscCdcBackend.hpp    ← 新建，实现 Backend 接口
```

```cpp
// 从 UART 切换到 USB CDC 只改一行：
// static MicroROS_UartBackend uart_backend;
static MicroROS_UscCdcBackend usb_backend;
MicroROS_Transport::GetInstance().SetBackend(usb_backend);
```

Transport 和状态机逻辑无需任何修改。

---

## 七、实施步骤

1. 在 `MicroROS_Transport.hpp` 中定义 `Backend` 抽象类，添加 `SetBackend()` 方法
2. 将构造函数中的 UART 初始化逻辑移出，改为通过 `backend_->OnTxComplete()` / `OnRxData()` 回调桥接
3. `TryTransmit()` 改为 `backend_->Send()`；`Open()`/`Close()` 代理到 backend
4. 移除 `dma_buffer_` 成员和 `#include "Bus/UART_Base.hpp"`
5. 新建 `MicroROS_UartBackend.hpp`，包含原 UART 逻辑
6. 修改 `MicroROS_Manager.hpp`：include UartBackend，构造时注入
7. 编译验证：3 个 preset 编译 0 error 0 warning
8. `arm-none-eabi-nm` 验证：Transport 符号表中不再出现 `UART_Base` 相关符号
9. （可选）运行时验证：`uxTaskGetStackHighWaterMark` 确认栈使用无恶化

---

## 八、与已有方案的主要差异

| 点 | 已有方案 | 本方案补充 |
|------|---------|-----------|
| 虚函数 vs 模板 | 仅虚函数 | 分析了两种方案，建议先用虚函数 |
| 初始化时序 | 一段简述 | 详细分析 DoubleBuffer 缓冲假设 |
| `tx_busy_` 归属 | 隐含 | 明确应该留在 Transport 层 |
| 接口位置 | 未讨论 | 建议 Backend 嵌套在 Transport 内 |
| 验证 | 编译 + nm | 补充了运行时栈监控验证 |
