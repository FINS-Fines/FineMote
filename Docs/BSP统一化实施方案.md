# BSP 统一化实施方案

> 日期：2026-07-14
> 范围：BSP/MC_Board/、BSP/MC_Board_02/、BSP/Robomaster_C/ 中的 FineMote_BSP/
> 目标：消除 3 份板卡间 BSP 文件的冗余 copy-paste，提取公共模板到 BSP/Common/

---

## 一、现状

通过 MD5 逐文件对比确认的现状：

| 文件 | MC_Board | MC_Board_02 | Robomaster_C | 差异 |
|------|----------|-------------|--------------|------|
| BSP_UART.h | 基准 | + SBUS DMA 特化 (3行) | = MC_Board | H7 板 UART\<3\> 用 DMA 接收 25 字节 |
| BSP_UART.cpp | 基准 | = MC_Board | = MC_Board | **三份 MD5 完全一致** |
| BSP_CAN.h | bxCAN | FDCAN (完全不同) | = MC_Board | 两套 HAL API：CAN_ vs FDCAN_ |
| BSP_CAN.cpp | bxCAN 回调 | FDCAN 回调 | = MC_Board | 回调函数完全不同 |
| BSP_PWM.h | 基准 | = MC_Board (MD5一致) | 仅缩进不同 | **语义完全一致** |
| BSP_RS485.h | GPIO 流控有效 | GPIO 流控注释 | 无此文件 | H7 版 Switch2Rx/Switch2Tx 为空 |
| BSP_RS485.cpp | 基准 | = MC_Board | 无此文件 | **两份 MD5 完全一致** |

当前 BSP 的 include 链：
```
Services/Bus/UART_Base.hpp: #include "BSP_UART.h"
  → 解析到 BSP/${BOARD_NAME}/FineMote_BSP/BSP_UART.h
    → #include "Board.h"
      → #include "{BoardName}.h"  (定义 BSP_UARTList, BSP_CANList 等)
```

---

## 二、目标文件结构

```
BSP/
├── Common/                          # 【新建】共享 BSP 代码
│   ├── BSP_Common_UART.h            # UART 公共模板
│   ├── BSP_Common_UART.cpp          # UART HAL 回调（原3份合一）
│   ├── BSP_Common_CAN.h             # CAN 统一模板（bxCAN/FDCAN #if 分支）
│   ├── BSP_Common_CAN.cpp           # CAN HAL 回调（#if 分支）
│   ├── BSP_Common_PWM.h             # PWM 公共模板
│   ├── BSP_Common_RS485.h           # RS485 公共模板
│   └── BSP_Common_RS485.cpp         # RS485 公共源文件
│
├── MC_Board/
│   ├── MC_Board.h                   # 【修改】增加宏定义
│   ├── CMakeLists.txt               # 【修改】增加 Common 路径
│   └── FineMote_BSP/
│       ├── BSP_UART.h               # 【修改】→ 薄 wrapper
│       ├── BSP_UART.cpp             # 【删除】
│       ├── BSP_CAN.h                # 【修改】→ 薄 wrapper
│       ├── BSP_CAN.cpp              # 【删除】
│       ├── BSP_PWM.h                # 【修改】→ 薄 wrapper
│       ├── BSP_RS485.h              # 【修改】→ 薄 wrapper
│       └── BSP_RS485.cpp            # 【删除】
│
├── MC_Board_02/
│   ├── MC_Board_02.h                # 【修改】增加宏定义
│   ├── CMakeLists.txt               # 【修改】同上
│   └── FineMote_BSP/
│       ├── BSP_UART.h               # 【修改】→ 薄 wrapper + SBUS DMA 特化
│       ├── BSP_UART.cpp             # 【删除】
│       ├── BSP_CAN.h                # 【修改】→ 薄 wrapper
│       ├── BSP_CAN.cpp              # 【删除】
│       ├── BSP_PWM.h                # 【修改】→ 薄 wrapper
│       ├── BSP_RS485.h              # 【修改】→ 薄 wrapper
│       └── BSP_RS485.cpp            # 【删除】
│
└── Robomaster_C/
    ├── Robomaster_C.h               # 【修改】增加宏定义
    ├── CMakeLists.txt               # 【修改】同上
    └── FineMote_BSP/
        ├── BSP_UART.h               # 【修改】→ 薄 wrapper
        ├── BSP_UART.cpp             # 【删除】
        ├── BSP_CAN.h                # 【修改】→ 薄 wrapper
        ├── BSP_CAN.cpp              # 【删除】
        └── BSP_PWM.h                # 【修改】→ 薄 wrapper
```

汇总：**新建 7 个，修改 20 个，删除 11 个**。

---

## 三、关键设计决策：`#if` 而非 `if constexpr`

评估报告中建议用 `if constexpr` + policy 模式。经分析，对 BSP_CAN **必须使用 `#if` 预处理分支**，原因：

1. bxCAN 类型（`CAN_HandleTypeDef`, `CAN_RxHeaderTypeDef`, `CAN_TxHeaderTypeDef`）定义在 `stm32f4xx_hal_can.h`，**仅 F4 HAL 中存在**
2. FDCAN 类型（`FDCAN_HandleTypeDef`, `FDCAN_RxHeaderTypeDef`, `FDCAN_TxHeaderTypeDef`）定义在 `stm32h7xx_hal_fdcan.h`，**仅 H7 HAL 中存在**
3. `if constexpr` 虽然编译期求值，但**未采用分支中的代码仍必须语法合法且类型可见**
4. 在 F4 编译时，`FDCAN_RxHeaderTypeDef` 等类型根本不存在 → 编译失败

因此定义 `#define BSP_CAN_IS_FDCAN 0/1` 宏，`BSP_Common_CAN.h` 和 `BSP_Common_CAN.cpp` 均使用 `#if BSP_CAN_IS_FDCAN`。

---

## 四、各公共文件内容来源与策略

### 4.1 BSP_Common_UART.h（新建）

从 MC_Board 的 BSP_UART.h 提取（不含 SBUS 特化的干净版本）。内容不变，include guard 改为 `FINEMOTE_BSP_COMMON_UART_H`。包含 `BSP_UARTs` 类、`BSP_UART<ID>` 模板类、默认 `Transmit()`/`Receive()` 实现。

### 4.2 BSP_Common_UART.cpp（新建）

从 MC_Board 的 BSP_UART.cpp 直接复制（三份 MD5 一致）。HAL UART 回调全部委托给 `FineMoteAux_UART<>`。

### 4.3 BSP_Common_CAN.h（新建）

融合 MC_Board（bxCAN）和 MC_Board_02（FDCAN）的 BSP_CAN.h。核心结构：

```cpp
template<uint8_t ID>
class BSP_CAN {
public:
    static BSP_CAN& GetInstance();
    void Transmit(FineMote_CAN_HeaderTypeDef*, uint8_t*);
    void Receive(FineMote_CAN_HeaderTypeDef*, uint8_t*);
private:
    BSP_CAN() { /* static_assert + BSP_CANs::GetInstance() + BSP_CAN_Setup() */ }
    void PeriphralInit() {
#if BSP_CAN_IS_FDCAN
        // FDCAN: HAL_FDCAN_ActivateNotification + FDCAN_FilterTypeDef + HAL_FDCAN_Start
#else
        // bxCAN: HAL_CAN_ActivateNotification + CAN_FilterTypeDef + HAL_CAN_Start
#endif
    }
    void BSP_CAN_Setup() { PeriphralInit(); }
};

// Receive 模板实现
template<uint8_t ID>
void BSP_CAN<ID>::Receive(...) {
#if BSP_CAN_IS_FDCAN
    // FDCAN_RxHeaderTypeDef + HAL_FDCAN_GetRxMessage → FineMote_CAN_HeaderTypeDef
#else
    // CAN_RxHeaderTypeDef + HAL_CAN_GetRxMessage → FineMote_CAN_HeaderTypeDef
#endif
}

// Transmit 模板实现
template<uint8_t ID>
void BSP_CAN<ID>::Transmit(...) {
#if BSP_CAN_IS_FDCAN
    // FineMote_CAN_HeaderTypeDef → FDCAN_TxHeaderTypeDef → HAL_FDCAN_AddMessageToTxFifoQ
#else
    // FineMote_CAN_HeaderTypeDef → CAN_TxHeaderTypeDef → HAL_CAN_AddTxMessage
#endif
}
```

只 include `"Bus/CAN_Header.hpp"`（最小正确 include）。

### 4.4 BSP_Common_CAN.cpp（新建）

融合两个版本的 BSP_CAN.cpp，`#if BSP_CAN_IS_FDCAN` 分支：

```cpp
#if BSP_CAN_IS_FDCAN
void HAL_FDCAN_RxFifo0Callback(FDCAN_HandleTypeDef*, uint32_t) { FineMoteAux_CAN<>::OnRxComplete(...); }
void HAL_FDCAN_RxFifo1Callback(FDCAN_HandleTypeDef*, uint32_t) { FineMoteAux_CAN<>::OnRxComplete(...); }
void HAL_FDCAN_TxFifoEmptyCallback(FDCAN_HandleTypeDef*) { FineMoteAux_CAN<>::OnTxComplete(...); }
#else
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef*) { FineMoteAux_CAN<>::OnRxComplete(...); }
void HAL_CAN_TxMailbox0CompleteCallback(CAN_HandleTypeDef*) { FineMoteAux_CAN<>::OnTxComplete(...); }
void HAL_CAN_TxMailbox1CompleteCallback(CAN_HandleTypeDef*) { FineMoteAux_CAN<>::OnTxComplete(...); }
void HAL_CAN_TxMailbox2CompleteCallback(CAN_HandleTypeDef*) { FineMoteAux_CAN<>::OnTxComplete(...); }
#endif
```

### 4.5 BSP_Common_PWM.h（新建）

从 MC_Board 的 BSP_PWM.h 直接复制（与 MC_Board_02 MD5 一致，Robomaster_C 仅缩进不同）。include guard 改为 `FINEMOTE_BSP_COMMON_PWM_H`。

### 4.6 BSP_Common_RS485.h / BSP_Common_RS485.cpp（新建）

从 MC_Board 版本提取。两板的差异仅在 `Switch2Rx`/`Switch2Tx` 中 GPIO 写操作是否注释掉。使用独立宏控制：

```cpp
static void Switch2Rx() {
#if BSP_RS485_FLOW_CONTROL_ENABLED
    HAL_GPIO_WritePin(BSP_RS485FlowCtrlPortList[ID], BSP_RS485FlowCtrlPinList[ID], GPIO_PIN_RESET);
#endif
}
static void Switch2Tx() {
#if BSP_RS485_FLOW_CONTROL_ENABLED
    HAL_GPIO_WritePin(BSP_RS485FlowCtrlPortList[ID], BSP_RS485FlowCtrlPinList[ID], GPIO_PIN_SET);
#endif
}
```

---

## 五、板级 {BoardName}.h 新增宏

```cpp
// MC_Board.h
#define BSP_CAN_IS_FDCAN 0
#define BSP_RS485_FLOW_CONTROL_ENABLED 1

// MC_Board_02.h
#define BSP_CAN_IS_FDCAN 1
#define BSP_RS485_FLOW_CONTROL_ENABLED 0

// Robomaster_C.h （无 RS485 模块，不定义 RS485 宏）
#define BSP_CAN_IS_FDCAN 0
```

---

## 六、板级薄文件示例

### MC_Board / Robomaster_C 的 BSP_UART.h（二者完全相同）

```cpp
#ifndef FINEMOTE_BSP_UART_H
#define FINEMOTE_BSP_UART_H
#include "BSP_Common_UART.h"
#endif
```

### MC_Board_02 的 BSP_UART.h（保留 SBUS DMA 特化）

```cpp
#ifndef FINEMOTE_BSP_UART_H
#define FINEMOTE_BSP_UART_H
#include "BSP_Common_UART.h"

// SBUS 接收端口需要用 DMA，固定 25 字节
template<> inline void BSP_UART<3>::Receive(uint8_t *data, uint16_t size) {
    HAL_UARTEx_ReceiveToIdle_DMA(BSP_UARTList[3], data, 25);
}
#endif
```

### 所有板 BSP_CAN.h（三块板完全相同）

```cpp
#ifndef BSP_CAN_H
#define BSP_CAN_H
#include "BSP_Common_CAN.h"
#endif
```

### 所有板 BSP_PWM.h（三块板完全相同）

```cpp
#ifndef FINEMOTE_BSP_PWM_H
#define FINEMOTE_BSP_PWM_H
#include "BSP_Common_PWM.h"
#endif
```

### MC_Board / MC_Board_02 的 BSP_RS485.h（二者相同）

```cpp
#ifndef FINEMOTE_BSP_RS485_H
#define FINEMOTE_BSP_RS485_H
#include "BSP_Common_RS485.h"
#endif
```

---

## 七、CMakeLists.txt 修改

三个板的 `BSP/{BoardName}/CMakeLists.txt` 做相同的两处修改：

**修改 1** —— include 路径增加 Common：
```cmake
target_include_directories(finemote_board INTERFACE
    ${CMAKE_CURRENT_SOURCE_DIR}
    ${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP
    ${PROJECT_SOURCE_DIR}/BSP/Common          # 【新增】
    ${CMAKE_CURRENT_SOURCE_DIR}/Middlewares/Third_Party/FreeRTOS_POSIX
)
```

**修改 2** —— 源文件 GLOB 增加 Common：
```cmake
file(GLOB _FINEMOTE_BSP_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/*.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP/*.c"
    "${CMAKE_CURRENT_SOURCE_DIR}/FineMote_BSP/*.cpp"
    "${PROJECT_SOURCE_DIR}/BSP/Common/*.c"    # 【新增】
    "${PROJECT_SOURCE_DIR}/BSP/Common/*.cpp"  # 【新增】
)
```

**不修改**：根 CMakeLists.txt、cmake/stm32cubemx/、CMakePresets.json、Services/Bus/ 下所有文件均不需要改动。

---

## 八、实施顺序

1. 创建 `BSP/Common/` 目录和 7 个公共文件
2. 修改 3 个板级 `{BoardName}.h`，增加宏定义
3. 修改 3 个板级 `CMakeLists.txt`
4. 将板级 `BSP_*.h` 改写为薄文件，删除板级 `BSP_*.cpp`
5. 编译验证

---

## 九、验证策略

1. **全配置编译通过**：3 块板 × 2 工具链（arm-none-eabi-gcc / armclang）= 6 个 preset，0 error 0 warning
2. **符号一致性**：`arm-none-eabi-nm` 对比修改前后 .elf 文件的 BSP 相关符号（`BSP_UART`、`BSP_CAN`、`BSP_PWM`、`BSP_RS485`、`FineMoteAux`），期望完全一致
3. **特定场景手动审查**：
   - MC_Board_02: `BSP_UART<3>::Receive` → `HAL_UARTEx_ReceiveToIdle_DMA`（非 IT 版本），长度固定 25
   - MC_Board_02: 编译 `HAL_FDCAN_*` 回调，不是 `HAL_CAN_*`
   - MC_Board / Robomaster_C: 编译 `HAL_CAN_*` 回调
   - MC_Board: RS485 流控 GPIO 写代码存在；MC_Board_02: 流控不产生 GPIO 代码
   - Robomaster_C: 无 RS485 模块，不参与 RS485 编译

---

## 十、潜在风险与对策

| 风险 | 概率 | 影响 | 对策 |
|------|------|------|------|
| `#if BSP_CAN_IS_FDCAN` 宏未定义 | 中 | 编译失败 | 在 Common 文件中加 `#ifndef` → `#error` 保护 |
| MC_Board_02 SBUS 特化与 Common 模板 ODR 冲突 | 中 | 链接错误 | 显式特化 `template<>` 优先级高于主模板，只要在 include 之后定义即可 |
| BSP_Common 文件中的 `#include "Board.h"` 解析到错误板 | 极低 | 编译失败 | CMake include 路径保证先搜索板级 FineMote_BSP，`Board.h` 在板级根目录 |
