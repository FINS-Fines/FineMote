# BMS I2C 设备统一调度改造记录


## PCF8574 P7 power_en telemetry

- `Components/BMS/BMS_PowerEnable.hpp` decodes P7 (bit 7): high means power connected; low means power disconnected and emergency stop pressed.
- Decoding is valid only when `BMS_SourcePcf8574` is present in `validMask`; invalid samples report `power_en=-1` and an unknown state instead of treating zero as an emergency stop.
- The sensor homepage and `/json` now expose the raw level and translated state. The JSON fields are `power_en_valid`, `power_en`, `power_en_level`, and `power_en_state` under `pcf8574`.
- When micro-ROS is enabled, `bms/power_en` publishes `std_msgs/msg/String` through the existing UART transport. Its JSON payload uses the same four fields as the webpage and is read directly from the thread-safe BMS snapshot.
- Hardware acceptance should cover P7 high/low, PCF8574 disconnect/recovery, and all-source invalid states; this telemetry does not replace the hardware emergency-stop circuit.
## 目标

PCF8574、ADS1115、BQ25756 遵循 FineMote 设备模型：设备构造时统一注册到
`DeviceScheduler`，由调度器按周期调用。设备驱动只采集并发布单一来源事件，滤波聚合和
BMS 业务处理在其他任务中完成。

不再保留独立的 `bms_i2c_acquire` 周期任务，也不在 `DeviceBase` 构造函数中提供
“是否自动注册”参数。

## 数据流

改造前：

```text
bms_i2c_acquire (CPU1)
  -> 依次直接调用三个设备
  -> FreeRTOS rawSampleQueue
  -> bms_i2c_filter
  -> BMS::SubmitSample()
  -> POSIX mq /bms_samples
  -> BMS pthread
  -> 状态、曲线、网页
```

当前实现：

```text
DeviceScheduler (CPU0)
  -> PCF8574 / ADS1115 / BQ25756 Update()、Handle()
  -> 单一来源 BMS_Sample 事件
  -> FreeRTOS rawSampleQueue
  -> bms_i2c_filter (CPU1，聚合并滤波)
  -> BMS::SubmitSample()
  -> POSIX mq /bms_samples
  -> BMS pthread (CPU1)
  -> 状态、曲线、网页
```

采集周期归 `DeviceScheduler` 管理；采集结果通过队列发送给滤波任务。滤波、状态聚合、
充电管理和网页发布不在调度线程中执行。

## 固定约束

1. `DeviceBase` 构造时无条件注册，所有设备实例必须在 `DeviceScheduler::Start()` 前构造。
2. 驱动通过采样回调暴露结果，不依赖 `BMS_Sample`、BMS 业务类型或 FreeRTOS 队列。
3. BSP 回调每次只生成一个来源的事件，多个设备不能并发修改同一个样本对象。
4. `updatedMask` 表示本次事件涉及的来源，`validMask` 表示聚合后的当前有效来源。
5. 某来源失败时，只清除该来源的有效位、缓存值和滤波器，不能影响其他来源。
6. PCF8574 和 ADS1115 周期为 250 ms，BQ25756 周期为 1000 ms。
7. 掉线后约 60 秒重试，避免 I2C NACK 和重复日志刷屏；恢复时只记录一次恢复日志。
8. FreeRTOS 队列仅存在于 ESP32 BSP；`Components/BMS` 保持 POSIX pthread、mutex 和
   mqueue 边界。
9. 网页必须根据来源和字段有效位显示状态，不能通过数值是否为 0 判断设备在线。

## 调度与队列实现

- 三个设备的调度周期写入 `DeviceBase::divisionFactor`，BSP 使用
  `pdMS_TO_TICKS()` 转换毫秒周期。
- `Update()` 采集并保存本轮结果，`Handle()` 通过回调向 BSP 发布。
- 250 ms 和 1000 ms bucket 可能并发生产事件；FreeRTOS 队列负责生产者并发安全。
- 队列满时丢弃最旧样本、保留最新样本，并使用原子计数器累计 drop 数量。
- `bms_i2c_filter` 持久保存聚合样本，只修改 `updatedMask` 指定的来源。
- 设备未设置回调时不主动采集。启动失败时不会持续向无人消费的队列写入，但设备实例仍已
  注册并受调度器生命周期约束。

## I2C 总线锁与 BQ25756 设备锁

底层 `I2C_Base<1>` mutex 只负责串行化共享总线上的单次 I2C 事务。它不能保证 BQ25756
“多寄存器配置”或“读取整组测量值”不会被另一个线程插入。

`BQ25756` 因此使用独立的 `BQ25756_OperationMutex` 和
`BQ25756_OperationGuard`，串行化以下高层入口：

- `ReadMeasurements()`；
- `Configure()`；
- `StartCharging()`；
- `StopCharging()`；
- `ResetWatchdog()`。

该锁是设备锁，不是总线锁。上述入口覆盖完整高层操作，内部细粒度寄存器函数不重复加设备锁。
禁止在此处复用受 BSP 条件编译控制的 `I2C_LockGuard`。

细粒度寄存器接口仍可用于初始化和诊断，但跨线程业务必须通过已加锁的高层入口，不能组合多个
细粒度接口绕过设备锁。

## BQ25756 在线状态与电池容量修正

### 原问题

1. BQ25756 已连接，但网页显示“待连接”。
2. 未充电时，电池电压和 6S 电压法 SOC 显示为 0。
3. 电池容量区域重复显示电池电流。

原来的 `ReadMeasurements()` 将电池电压、电流、输入测量、温度、反馈电压、充电状态、故障和
看门狗视为不可分割的整包。任一可选字段失败都会使整个 BQ 来源失效。TS 未连接、温度无法换算
或辅助寄存器读取失败，都会让网页误判设备离线，电池估算器也收不到有效电压。

启动阶段还存在依赖循环：BQ25756 ADC 原先在充电配置时启用，但充电配置又要求先取得有效
BQ 样本。

### 状态语义

| 状态 | 判据 | 网页行为 |
| --- | --- | --- |
| 设备在线 | 本轮至少一个 BQ 寄存器事务成功 | 显示“已连接” |
| 电压有效 | `validFields` 包含 `BatteryVoltage` | 显示电压并更新 SOC |
| 正在充电 | PCF8574 组合状态为 `Charging` | 显示“充电中” |
| 在线但未充电 | BQ 在线且 PCF 状态为非充电 | 显示“未充电”，仍显示电压和 SOC |
| 设备离线 | 本轮所有 BQ 通信均失败 | 显示“待连接”，约 60 秒后重试 |

设备在线、字段有效、正在充电是三个独立维度。不得用“未充电”推断设备离线，也不得用数值 0
推断字段无效。

### 修改手段

- `BQ25756_Measurements` 增加 `validFields`，逐项记录测量字段有效性。
- 可选温度或辅助字段失败不再使整颗器件离线；所有寄存器事务均失败才返回采样失败。
- 增加 `InitializeMeasurements()`，启动时只初始化 ADC 和测量通道，不写充电电压、电流，
  也不改变充电启停状态。
- `BMS_ChargerMeasurements` 继续传递字段有效位；滤波任务只处理有效字段。
- 电池估算器只要求电池电压有效即可更新电压和 SOC；电流无效时停止安时积分。
- BQ 掉线或电压失效时，当前电压和 SOC 失效，并断开积分时间连续性；累计统计保留。
- 充电曲线仍要求电池电压和电流同时有效，避免产生半有效曲线点。
- 网页容量区删除电池电流；电压或 SOC 无效时显示等待状态，不再显示伪造的 0。
- `/json` 保留原电流字段以兼容调用者，并增加电压、电流等有效位。

## 代码组织

PCF8574、ADS1115 和 BQ25756 的类体只保留接口声明、常量和成员布局；模板函数实现位于同一
`.hpp` 文件的类定义之后。模板定义必须在使用点可见，因此不能直接移动到普通 `.cpp` 文件。

默认参数和 `final` 说明符只写在类内声明中。此调整只改善可读性，不改变寄存器访问顺序、
调度周期、失败重试、回调或数据流。

## 审查重点

- 三个设备共享 I2C1，所有底层事务必须经过同一个 `I2C_Base<1>` mutex。
- ADS1115 四通道单次转换包含轮询等待，需要观察调度 bucket 的执行时间和 deadline。
- BQ25756 控制操作来自 BMS pthread，测量来自 DeviceScheduler，必须由设备锁串行化。
- `validMask` 表示来源在线，`charger.validFields` 表示 BQ 内部字段可用，二者不能混用。
- 未充电时应看到：BQ25756 已连接、状态为未充电/待机、电池电压为实测值、SOC 为估算值。
- TS 未连接时允许温度无效，但电池电压有效时 BQ 仍应在线。
- `Interface/UserTasks/UserTask1.cpp` 中另有受宏控制的 PCF8574 测试实例。未来启用时必须避免
  与 BMS 实例同时访问地址 `0x20`。

## 已完成项

- [x] 删除三个设备构造函数的 `registerWithScheduler` 参数。
- [x] 由三个设备实现调度采集和结果回调。
- [x] 删除独立的 `bms_i2c_acquire` 任务。
- [x] 滤波任务聚合单一来源事件并维护有效位。
- [x] 保留掉线退避与恢复日志。
- [x] 使用 BQ25756 设备锁串行化测量和控制操作。
- [x] 修正 BQ 在线状态、字段有效性和未充电时的电压/SOC 显示。
- [x] 整理三个模板设备头文件的声明和实现布局。
- [x] 创建中文 `finemote-bms` skill。

## 验证边界

已对 BMS、BSP 聚合、网页桥接等相关源文件执行定向编译检查。后续硬件审查应重点验证：

- 250 ms 与 1000 ms 调度周期和实际耗时；
- 队列溢出策略与 drop 计数；
- 三个器件独立掉线和恢复；
- BQ25756 控制与采集并发；
- 未充电、TS 未连接和部分寄存器失败时的在线状态；
- 电池电压、SOC、充电状态和 JSON 有效位的一致性。
