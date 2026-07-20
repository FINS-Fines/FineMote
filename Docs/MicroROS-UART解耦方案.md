# MicroROS 与 UART 解耦方案

> 日期：2026-07-14
> 范围：Services/MicroROS/MicroROS_Transport.hpp、MicroROS_Manager.hpp
> 目标：将 MicroROS_Transport 与 UART 硬件的耦合解开，使传输层可替换（USB CDC、SPI 等）

---

## 一、现状：3 层耦合

当前 `MicroROS_Transport.hpp` 与 UART 的耦合点：

| 层次 | 行号 | 耦合方式 |
|------|------|---------|
| 头文件依赖 | L14 | `#include "Bus/UART_Base.hpp"` — 整个 Bus 层都拖进来了 |
| 构造层 | L75-86 | 构造函数直接调 `UART_Base<MICRO_ROS_UART_ID>::GetInstance().BindTxHandle(...)` |
| 发送路径 | L122 | `TryTransmit()` 调 `UART_Base<...>::GetInstance().Transmit(...)` |
| 接收路径 | L145 | 成员变量 `UARTBuffer<MICRO_ROS_UART_ID, 512> dma_buffer_` — 类型级耦合 |

耦合链：
```
MicroROS_Transport  →  UART_Base<ID>  →  BSP_UART<ID>  →  HAL_UART_xxx
       ↕ (DMA双缓冲)
   UARTBuffer<ID,N>  →  DoubleBuffer<N>
```

---

## 二、解耦设计

### 核心思路

在 `MicroROS_Transport` 内部定义一个抽象 `Backend` 接口，Transport 只依赖接口，不依赖具体 UART 实现。

```
MicroROS_Transport  →  Backend (抽象接口)
                           ↑
                      implements
                           ↑
              MicroROS_UartBackend  →  UART_Base / UARTBuffer / BSP_UART
```

### Backend 接口

micro-ROS 自定义传输本质上只需要 4 个操作。将其抽象为 Backend 接口：

```cpp
class Backend {
public:
    virtual ~Backend() = default;

    // 生命周期（对应 micro-ROS 的 open/close）
    virtual bool Open()  { return true; }
    virtual bool Close() { return true; }

    // 发送数据，返回 true 表示发送已启动
    virtual bool Send(const uint8_t* data, size_t len) = 0;

    // 注册回调：TX 完成时调用（用于清除 tx_busy_ 标志）
    virtual void OnTxComplete(std::function<void()> callback) = 0;

    // 注册回调：RX 数据到达时调用（用于写入环形缓冲区）
    virtual void OnRxData(std::function<void(const uint8_t*, size_t)> callback) = 0;
};
```

### MicroROS_Transport 改造

- **移除** `#include "Bus/UART_Base.hpp"`
- **移除** 成员 `UARTBuffer<...> dma_buffer_`
- **移除** 构造函数中对 `UART_Base::BindTxHandle()` 的调用
- **新增** 成员 `Backend* backend_` + `void SetBackend(Backend&)` 方法
- `TryTransmit()` 改为调用 `backend_->Send()`
- `Open()` / `Close()` 代理到 backend
- 环形缓冲区 `rx_buf_`、`tx_busy_` 等协议层逻辑全部保留在 Transport 中

### MicroROS_UartBackend（新建文件）

```cpp
// Services/MicroROS/MicroROS_UartBackend.hpp
// 
// 持有 UARTBuffer（DMA 双缓冲），实现 Backend 接口，
// 将 UART 的 TX/RX 事件桥接到 Transport 的回调。

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

    void OnTxComplete(std::function<void()> callback) override {
        tx_callback_ = std::move(callback);
    }

    void OnRxData(std::function<void(const uint8_t*, size_t)> callback) override {
        rx_callback_ = std::move(callback);
    }

private:
    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;
    std::function<void()> tx_callback_;
    std::function<void(const uint8_t*, size_t)> rx_callback_;
};
```

### MicroROS_Manager 装配

Manager 构造函数中完成依赖注入：

```cpp
MicroROS_Manager()
{
    static MicroROS_UartBackend uart_backend;           // 新建
    MicroROS_Transport::GetInstance().SetBackend(uart_backend);  // 注入
    rmw_uros_set_custom_transport(true, nullptr,
                                  MicroROS_Transport::Open,
                                  MicroROS_Transport::Close,
                                  MicroROS_Transport::Write,
                                  MicroROS_Transport::Read);
    allocator_ = rcl_get_default_allocator();
    StartThread();
}
```

---

## 三、涉及文件

| 文件 | 动作 | 说明 |
|------|------|------|
| `Services/MicroROS/MicroROS_Transport.hpp` | **修改** | 移除 UART 依赖，新增 Backend 接口 + SetBackend |
| `Services/MicroROS/MicroROS_UartBackend.hpp` | **新建** | UART 具体实现，持有 UARTBuffer 和 UART_Base 调用 |
| `Services/MicroROS/MicroROS_Manager.hpp` | **修改** | 构造函数中创建 UartBackend 并注入到 Transport |
| `Docs/MicroROS-UART解耦方案.md` | **新建** | 本文档 |

Bus 层（`UART_Base.hpp`、`BSP_UART.h` 等）**无需修改**。

---

## 四、迁移前后对比

### 改造前（MicroROS_Transport.hpp 部分）

```cpp
#include "Bus/UART_Base.hpp"   // ← 耦合

class MicroROS_Transport {
    MicroROS_Transport() :
        dma_buffer_([this](uint8_t* data, size_t size) {
            this->PushRxData(data, size);       // ← UARTBuffer 类型耦合
        })
    {
        UART_Base<MICRO_ROS_UART_ID>::GetInstance()
            .BindTxHandle([this]() { ... });    // ← UART_Base 直接调用
    }

    bool TryTransmit(const uint8_t* buf, size_t len) {
        // ...
        UART_Base<MICRO_ROS_UART_ID>::GetInstance()
            .Transmit(tx_buffer_, ...);          // ← UART_Base 直接调用
        return true;
    }

    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;  // ← 类型耦合
};
```

### 改造后

```cpp
// 不再 include Bus/UART_Base.hpp

class MicroROS_Transport {
public:
    class Backend { /* 纯抽象接口 */ };

    void SetBackend(Backend& backend) {
        backend_ = &backend;
        backend_->OnTxComplete([this]() { tx_busy_.store(false, ...); });
        backend_->OnRxData([this](const uint8_t* d, size_t s) { PushRxData(d, s); });
    }

    static bool Open(struct uxrCustomTransport* t) {
        GetInstance().ResetRx();
        return GetInstance().backend_->Open();
    }

private:
    bool TryTransmit(const uint8_t* buf, size_t len) {
        // ...
        return backend_->Send(tx_buffer_, len);  // 委托给抽象接口
    }

    Backend* backend_ = nullptr;
    // 不再有 UARTBuffer 成员
};
```

---

## 五、替换为新传输层（示例）

解耦完成后，若要用 USB CDC 替代 UART，只需：

1. 实现 `MicroROS_UscCdcBackend : public MicroROS_Transport::Backend`
2. 在 Manager 构造函数中改一行：

```cpp
// 改前
static MicroROS_UartBackend uart_backend;
MicroROS_Transport::GetInstance().SetBackend(uart_backend);

// 改后
static MicroROS_UscCdcBackend usb_backend;
MicroROS_Transport::GetInstance().SetBackend(usb_backend);
```

MicroROS_Transport 和 Manager 状态机逻辑无需任何修改。

---

## 六、关于初始化顺序

`MicroROS_UartBackend` 构造函数中 `UARTBuffer` 即启动 DMA 接收。此时 `rx_callback_` 尚未设置（为 nullptr），DMA 收到的数据会被 `UARTBuffer` 的 `DoubleBuffer` 内部缓冲。当 `SetBackend()` 注册回调后，数据开始流入 Transport 的环形缓冲区。

由于 `MicroROS_Manager` 构造函数中先创建 UartBackend 再 `StartThread()`，而线程的第一个循环是 `WAITING_AGENT` 状态（只 ping agent，不 spin），不存在竞态条件。

---

## 七、实施步骤

1. 在 `MicroROS_Transport.hpp` 中定义 `Backend` 抽象类，添加 `SetBackend()` 方法
2. 将构造函数中的 UART 初始化逻辑移出，改为依赖 `backend_->OnTxComplete()` / `OnRxData()` 回调
3. `TryTransmit()` 改为 `backend_->Send()`；`Open()`/`Close()` 代理到 backend
4. 移除 `dma_buffer_` 成员和 `#include "Bus/UART_Base.hpp"`
5. 新建 `MicroROS_UartBackend.hpp`，包含原来的 UART 逻辑
6. 修改 `MicroROS_Manager.hpp`：include UartBackend，构造时注入
7. 编译验证：确认所有 preset 编译通过、无新增 warning

## 八、验证

- 3 块板 × arm-none-eabi-gcc = 3 个 preset 编译 0 error 0 warning
- `arm-none-eabi-nm` 对比解耦前后 .elf 符号表，确认无意外符号变更
- 逻辑审查：`TryTransmit` → `backend_->Send()` → `UART_Base::Transmit()` 调用链正确
