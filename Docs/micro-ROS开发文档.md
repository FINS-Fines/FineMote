# micro\-ROS开发文档

> **项目**: FineMote — IWIN\-FINS Lab, 上海交通大学
> **平台**: STM32H7 \(Cortex\-M7\) \+ FreeRTOS
> **ROS 2 发行版**: Humble
> **最后更新**: 2026\-07\-03
> 
> 

---

## 目录

1. micro\-ROS 简介

2. micro\-ROS 架构

3. micro\-ROS 构建与移植

    - 3\.1 工具链配置

    - 3\.2 构建 micro\-ROS 静态库

    - 3\.3 将 micro\-ROS 集成到 MCU 项目

    - 3\.4 创建移植层（传输层 \+ 时钟 \+ 分配器）

4. FineMote micro\-ROS 实现

    - 4\.1 整体架构

    - 4\.2 通信概念：Publisher、Subscriber 与 QoS

    - 4\.3 自定义 UART 传输层

    - 4\.4 状态机管理

    - 4\.5 Agent 模型（Publisher/Subscriber 模板）

    - 4\.6 消息类型注册机制

    - 4\.7 StateSnapshot 机制

    - 4\.8 板级支持

5. 搭建与使用

    - 5\.1 上位机（Host PC）配置

    - 5\.2 下位机（MCU）配置

    - 5\.3 日常使用流程

6. 注意事项与踩坑记录

7. 参考资料

---

## 1\. micro\-ROS 简介

### 什么是 micro\-ROS？

[micro\-ROS](https://micro.vulcanexus.org/) 是一个开源框架，旨在将 ROS 2 的功能引入资源受限的微控制器（MCU）。如官方文档所述："micro\-ROS puts ROS 2 onto microcontrollers"——micro\-ROS 将 ROS 2 带到了微控制器上。

> **注意**: micro\-ROS 官方网站已于近期从 `micro.ros.org` 迁移至 `micro.vulcanexus.org`，旧域名会自动重定向。本文档中的链接已全部更新为新域名。
> 
> 

micro\-ROS 由 [OFERA](https://www.ofera.eu/)（Open Framework for Embedded Robotics Applications）项目发起，该项目由欧盟 Horizon 2020 资助，汇集了 Bosch、Fiware、PIAP、Acutronic Robotics 和 eProsima 等多家公司。目前由 [eProsima](https://www.eprosima.com/) 主导开发和维护。micro\-ROS 完全开源（Apache 2\.0 许可证），源代码在 [GitHub](https://github.com/micro-ROS/) 上可用。

从定位上看，micro\-ROS 是 **rosserial** 的改进版、ROS 2 兼容的替代品。rosserial 将 ROS 1 消息封装后通过串口传输，而 micro\-ROS 不仅仅是消息封装——它将 ROS 2 的完整客户端库（rcl）裁剪优化后搬到了 MCU 上，提供了原生的 Publisher/Subscriber API、QoS 支持和执行器模型。

### micro\-ROS 的关键特性

|特性|说明|
|---|---|
|**ROS 2 原生兼容**|基于 ROS 2 客户端库（rcl），而非简单的消息封装|
|**静态内存分配**|通过 rclc（优化的客户端库）尽可能使用静态内存，减少动态分配|
|**RTOS 支持**|原生支持 FreeRTOS 和 Zephyr，也可裸机运行|
|**DDS 中间件**|使用 eProsima 的 Micro\-XRCE\-DDS，专为资源受限设备优化的 DDS 实现|
|**多种传输方式**|支持 UART、UDP、TCP、USB CDC 及自定义传输层|
|**QoS 支持**|支持 Reliable 和 Best\-Effort 两种服务质量模式|

### micro\-ROS 的通信模型

micro\-ROS **不是独立的**——它需要一个运行在 ROS 2 主机上的 **micro\-ROS Agent**（代理/中间人）来管理与 MCU 客户端的通信。在 micro\-ROS 中：

- MCU 始终是 **Client**（客户端），从不作为 Server

- 两个 MCU 之间的通信必须经过 Agent 中转，不能直接互通

- 连接由 MCU 端发起，Agent 端被动接受

> **注意**: eProsima 正在开发 [Peer\-to\-Peer](https://github.com/eProsima/Micro-XRCE-DDS-Client/tree/feature/brokerless_p2p) 通信模式，以支持 MCU 之间的直接通信。
> 
> 

---

## 2\. micro\-ROS 架构

micro\-ROS 尽量复用了 ROS 2 客户端代码（来自 rcl 库），但做了优化以减少动态内存需求，尽可能使用静态内存分配，从而形成了优化的 **rclc** 库。底层通信标准是 DDS（与 ROS 2 相同），但使用的是 eProsima 专门为资源受限 MCU 优化的实现——[Micro\-XRCE\-DDS](https://micro-xrce-dds.docs.eprosima.com/en/latest/)。

一个完整的 micro\-ROS 项目由 4 个部分组成：

```Plain Text
┌─────────────────────────────────────────────────────┐
│                    Host PC (Linux)                   │
│  ┌───────────────────────────────────────────────┐  │
│  │              ROS 2 + micro-ROS Agent           │  │
│  │  (代理/中间人, 管理 MCU 与 ROS 2 之间的通信)     │  │
│  └──────────────────┬────────────────────────────┘  │
│                     │ UART / UDP / USB               │
└─────────────────────┼────────────────────────────────┘
                      │
┌─────────────────────┼────────────────────────────────┐
│   MCU (STM32)       │                                │
│  ┌──────────────────┴────────────────────────────┐  │
│  │         micro-ROS Client Application           │  │
│  │       (用户代码: Publisher, Subscriber 等)       │  │
│  ├───────────────────────────────────────────────┤  │
│  │         rclc (ROS 2 Client Library)            │  │
│  │         rmw_microxrcedds (中间件接口)           │  │
│  │         Micro-XRCE-DDS Client (DDS 实现)       │  │
│  ├───────────────────────────────────────────────┤  │
│  │         移植层 (Port Layer)                     │  │
│  │   - 自定义传输函数 (open/close/write/read)      │  │
│  │   - 时钟函数 (FreeRTOS-POSIX 提供)               │  │
│  │   - 内存分配器 (默认 malloc/free)                │  │
│  ├───────────────────────────────────────────────┤  │
│  │         FreeRTOS + HAL (底层平台)               │  │
│  └───────────────────────────────────────────────┘  │
└─────────────────────────────────────────────────────┘
```

### 四个组成部分

|部分|内容|位置|
|---|---|---|
|**micro\-ROS 静态库**|包含所有 micro\-ROS 代码、API 以及 DDS 通信中间件|`ThirdParty/micro-ROS/libmicroros_*.a`|
|**micro\-ROS Agent**|在 Host PC 上运行的代理，管理客户端通信|PC 端 ROS 2 工作区|
|**MCU 移植层**|底层传输函数 \+ 时钟函数|`Services/MicroROS/MicroROS_Transport.hpp`|
|**micro\-ROS 客户端应用**|用户的 Publisher/Subscriber 代码|`Services/MicroROS/MicroROS_Agent.hpp` 及各 Task 文件|

---

## 3\. micro\-ROS 构建与移植

将 micro\-ROS 移植到新 MCU 的过程分为四个步骤。本章节以 STM32H7 \(Cortex\-M7\) \+ FreeRTOS 为例。

### 3\.1 工具链配置

#### 前置条件

micro\-ROS 静态库的构建**强烈推荐在 Linux 环境下进行**（Ubuntu 22\.04 LTS）。Windows 上构建会遇到路径、工具链兼容性等诸多问题。构建完成后，生成的 `.a` 静态库和 `.h` 头文件可以拿到 Windows 下的 IDE（如 Keil、CLion）中直接使用。

- **操作系统**: Ubuntu 22\.04 LTS

- **ROS 2**: Humble Hawksbill

- **ARM 工具链**: `arm-none-eabi-gcc`（`apt install gcc-arm-none-eabi`）

```bash
# 安装 ROS 2 Humble
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 安装构建依赖
sudo apt install python3-colcon-common-extensions python3-pip
```

#### 创建 micro\-ROS 工作区

> 此步骤也可参照已有飞书文档：[micro\-ROS上位机配置步骤](https://my.feishu.cn/docx/T45fdeQb8o43qVxA3sncWSwWnVg)，但本文档使用 `generate_lib` 模式构建静态库（而非直接生成完整固件），以便更灵活地集成到 CMake 项目中。
> 
> 

```bash
# 1. 加载 ROS 2 环境
source /opt/ros/humble/setup.bash

# 2. 创建工作区并克隆 micro_ros_setup
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup

# 3. 安装 ROS 2 包依赖
sudo apt update && rosdep update
rosdep install --from-paths src --ignore-src -y

# 4. 构建 micro-ROS 工具链
colcon build
source install/local_setup.bash
```

> **说明**: `colcon build` 构建的是 micro\-ROS 的构建工具本身（`micro_ros_setup` 包），还不是最终的 MCU 静态库。这些工具提供了 `create_firmware_ws.sh`、`build_firmware.sh` 等脚本，用于下载源码、配置选项并交叉编译。
> 
> 

### 3\.2 构建 micro\-ROS 静态库

构建 micro\-ROS 静态库需要两个配置文件：

- **工具链文件** \(`.cmake`\)：指定交叉编译器、目标 CPU 架构、编译选项

- **colcon 元数据文件** \(`.meta`\)：指定 micro\-ROS 的功能选项和资源上限

#### 创建固件构建环境

```bash
# 加载环境（如果新开终端需要重新 source）
source /opt/ros/humble/setup.bash
source ~/microros_ws/install/local_setup.bash

# 创建静态库构建环境（generate_lib 模式）
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib
```

执行后，`microros_ws/firmware/` 目录下会生成完整的 micro\-ROS 源码构建树（包含 rcl、rclc、rmw\_microxrcedds、Micro\-XRCE\-DDS Client 等组件）。

#### 自定义工具链文件 \(`toolchain.cmake`\)

以下是 FineMote 项目实际使用的工具链配置，结构清晰、注释完备：

```cmake
set(CMAKE_SYSTEM_NAME Generic)
set(CMAKE_CROSSCOMPILING 1)
set(CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY)

set(CMAKE_C_COMPILER arm-none-eabi-gcc)
set(CMAKE_CXX_COMPILER arm-none-eabi-g++)
set(CMAKE_ASM_COMPILER arm-none-eabi-gcc)

# ============================================================
# 目标平台配置 —— 根据 MCU 核心修改此处
# ============================================================
# Cortex-M7: set(TARGET_CPU "cortex-m7")  set(TARGET_FPU "fpv5-sp-d16")
# Cortex-M4: set(TARGET_CPU "cortex-m4")  set(TARGET_FPU "fpv4-sp-d16")
# Cortex-M3: set(TARGET_CPU "cortex-m3")  (无 FPU, 用 -mfloat-abi=soft)
set(TARGET_CPU "cortex-m7")
set(TARGET_FPU "fpv5-sp-d16")

set(ARCH_CPU_FLAGS
    "-mcpu=${TARGET_CPU} -mthumb -mfpu=${TARGET_FPU} -mfloat-abi=hard"
)

set(CMAKE_C_COMPILER_WORKS 1 CACHE INTERNAL "")
set(CMAKE_CXX_COMPILER_WORKS 1 CACHE INTERNAL "")

# RET_CFLAGS 从环境变量注入（可在构建脚本中预设额外标志）
set(FLAGS $ENV{RET_CFLAGS} CACHE STRING "" FORCE)
set(MICROROSFLAGS "-DCLOCK_MONOTONIC=0 -D'__attribute__(x)='" CACHE STRING "" FORCE)

# ============================================================
# 优化选项 —— 包含 -fshort-wchar
# ============================================================
set(OPT_FLAGS
    "-funsigned-char -fshort-enums -fshort-wchar"
)

# C 和 C++ 最终标志拼接
string(CONCAT FINAL_C_FLAGS
    "${ARCH_CPU_FLAGS} "
    "${OPT_FLAGS} "
    "${MICROROS_EXTRA_FLAGS} "
    "-std=c11"
)
string(CONCAT FINAL_CXX_FLAGS
    "${ARCH_CPU_FLAGS} "
    "${OPT_FLAGS} "
    "${MICROROS_EXTRA_FLAGS} "
    "-std=c++14 -fno-exceptions"
)

set(CMAKE_C_FLAGS_INIT "${FINAL_C_FLAGS} ${FLAGS} ${MICROROSFLAGS}" CACHE STRING "" FORCE)
set(CMAKE_CXX_FLAGS_INIT "${FINAL_CXX_FLAGS} ${FLAGS} -fno-rtti ${MICROROSFLAGS}" CACHE STRING "" FORCE)

set(__BIG_ENDIAN__ 0)
```

##### 各选项详细说明

|选项|含义|注意事项|
|---|---|---|
|`CMAKE_SYSTEM_NAME Generic`|告知 CMake 这是裸机/RTOS 目标（非 Linux/Windows）|必须设置|
|`CMAKE_TRY_COMPILE_TARGET_TYPE STATIC_LIBRARY`|让 CMake 用静态库方式测试编译（而非链接可执行文件）|裸机编译必需|
|`CMAKE_C_COMPILER arm-none-eabi-gcc`|指定 ARM 交叉编译器|确保 `arm-none-eabi-gcc` 在 PATH 中|
|`-mcpu=cortex-m7`|目标 ARM 核心|**M7 和 M4 不可混用**|
|`-mfpu=fpv5-sp-d16`|浮点单元类型|M7→`fpv5-sp-d16`，M4→`fpv4-sp-d16`|
|`-mfloat-abi=hard`|硬件浮点 ABI|有 FPU 的 MCU 推荐 hard|
|`-mthumb`|使用 Thumb 指令集|ARM Cortex\-M 必须使用|
|`-funsigned-char`|`char` 默认为 `unsigned`|与 ARM 编译器习惯一致|
|`-fshort-enums`|枚举类型使用最小宽度|减小内存占用|
|`-fshort-wchar`|⚠️ `wchar_t` 使用 16 位（而非 GCC 默认的 32 位）|见下方详细说明|
|`-fno-exceptions`|禁用 C\+\+ 异常|嵌入式环境通常禁用|
|`-fno-rtti`|禁用 C\+\+ 运行时类型识别|减小代码体积|
|`-DCLOCK_MONOTONIC=0`|定义时钟常量（ARM 工具链 `time.h` 中缺失）|必须，否则编译报错|
|`-D'__attribute__(x)='`|禁用 GCC 属性语法|micro\-ROS 内部兼容性宏|
|`__BIG_ENDIAN__ 0`|小端模式（ARM Cortex\-M 默认）|—|

##### ⚠️ `-fshort-wchar` 的影响

**这是 micro\-ROS 移植中最容易出问题的编译选项之一。**

- micro\-ROS 库内部使用 `wchar_t` 类型，ARM GCC 默认 `wchar_t` 为 32 位（4 字节）

- STM32CubeMX 生成的代码和很多嵌入式项目使用 `-fshort-wchar`，将 `wchar_t` 变为 16 位（2 字节）

- **如果静态库编译时和 MCU 项目编译时的 ****`wchar_t`**** 宽度不一致**，会导致：

    - 链接错误（符号找不到，因为 `wchar_t` 的 name mangling 不同）

    - 运行时 ABI 不兼容（结构体布局不同，数据错乱）

**FineMote 的处理策略**: 静态库和项目代码**统一使用**`-fshort-wchar`（即两侧均启用此标志）。由于所有依赖 micro\-ROS 的 `.cpp` 文件编译时都带 `-fshort-wchar`，且静态库构建时也已启用，因此 ABI 保持一致。

> **注意**: 本项目历史上曾经历 `-fshort-wchar` 反复调整的过程：
> 
> - commit `56f140c`: 首次启用 `-fshort-wchar`
> 
> - commit `738d92c`: 曾短暂禁用以排查兼容性问题，最终确认统一启用
> 
> 

##### M4 和 M7 芯片的配置差异

|配置项|Cortex\-M7 \(STM32H7\)|Cortex\-M4 \(STM32F4/L4\)|
|---|---|---|
|`TARGET_CPU`|`cortex-m7`|`cortex-m4`|
|`TARGET_FPU`|`fpv5-sp-d16`|`fpv4-sp-d16`|
|`-mfloat-abi`|`hard`|`hard`（如有 FPU）|
|静态库文件|`libmicroros_m7.a`|`libmicroros_m4.a`|
|典型板卡|MC\_Board, MC\_Board\_02, FineMote|Robomaster\_C, Robomaster\_A|

> **关键原则**: 两个核心必须分别编译各自的 `libmicroros_*.a`，**不可混用**。M7 的库包含 M7 专用指令（如 DSP 扩展），在 M4 上会触发 UsageFault。
> 
> 

#### 自定义 micro\-ROS 配置 \(`colcon.meta`\)

以下是 FineMote 项目实际使用的 colcon 元数据文件，该配置针对 UART 自定义传输和嵌入式资源限制进行了精细化调整：

```yaml
{
    "names": {
        "tracetools": {
            "cmake-args": [
                "-DTRACETOOLS_DISABLED=ON",
                "-DTRACETOOLS_STATUS_CHECKING_TOOL=OFF"
            ]
        },
        "rosidl_typesupport": {
            "cmake-args": [
                "-DROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT=ON"
            ]
        },
        "rcl": {
            "cmake-args": [
                "-DBUILD_TESTING=OFF",
                "-DRCL_COMMAND_LINE_ENABLED=OFF",
                "-DRCL_LOGGING_ENABLED=OFF"
            ]
        },
        "rcutils": {
            "cmake-args": [
                "-DENABLE_TESTING=OFF",
                "-DRCUTILS_NO_FILESYSTEM=ON",
                "-DRCUTILS_NO_THREAD_SUPPORT=ON",
                "-DRCUTILS_NO_64_ATOMIC=ON",
                "-DRCUTILS_AVOID_DYNAMIC_ALLOCATION=ON"
            ]
        },
        "microxrcedds_client": {
            "cmake-args": [
                "-DUCLIENT_PIC=OFF",
                "-DUCLIENT_PROFILE_UDP=OFF",
                "-DUCLIENT_PROFILE_TCP=OFF",
                "-DUCLIENT_PROFILE_DISCOVERY=OFF",
                "-DUCLIENT_PROFILE_SERIAL=OFF",
                "-DUCLIENT_PROFILE_STREAM_FRAMING=ON",
                "-DUCLIENT_PROFILE_CUSTOM_TRANSPORT=ON"
            ]
        },
        "rmw_microxrcedds": {
            "cmake-args": [
                "-DRMW_UXRCE_MAX_NODES=3",
                "-DRMW_UXRCE_MAX_PUBLISHERS=10",
                "-DRMW_UXRCE_MAX_SUBSCRIPTIONS=5",
                "-DRMW_UXRCE_MAX_SERVICES=1",
                "-DRMW_UXRCE_MAX_CLIENTS=1",
                "-DRMW_UXRCE_MAX_HISTORY=4",
                "-DRMW_UXRCE_TRANSPORT=custom",
                "-DUCLIENT_CUSTOM_TRANSPORT_MTU=1024",
                "-DUCLIENT_CUSTOM_TRANSPORT_MW_MTU=1024"
            ]
        }
    }
}
```

##### 各选项详细说明

|选项|值|说明|
|---|---|---|
|`TRACETOOLS_DISABLED`|`ON`|禁用追踪工具，嵌入式环境无需性能追踪|
|`ROSIDL_TYPESUPPORT_SINGLE_TYPESUPPORT`|`ON`|仅使用单一类型支持（减小体积）|
|`RCL_COMMAND_LINE_ENABLED`|`OFF`|禁用命令行接口，MCU 不需要|
|`RCL_LOGGING_ENABLED`|`OFF`|禁用日志系统（如需调试可临时开启）|
|`RCUTILS_NO_FILESYSTEM`|`ON`|无文件系统支持|
|`RCUTILS_NO_THREAD_SUPPORT`|`ON`|无原生线程库（由 FreeRTOS 提供）|
|`RCUTILS_NO_64_ATOMIC`|`ON`|禁用 64 位原子操作（Cortex\-M 不支持 64 位原子）|
|`RCUTILS_AVOID_DYNAMIC_ALLOCATION`|`ON`|尽可能避免动态内存分配|
|`UCLIENT_PROFILE_UDP/TCP/SERIAL`|`OFF`|禁用所有默认传输方式|
|`UCLIENT_PROFILE_CUSTOM_TRANSPORT`|`ON`|✅ 启用自定义传输（本项目使用 UART）|
|`UCLIENT_PROFILE_STREAM_FRAMING`|`ON`|启用流帧协议（串口通信必需）|
|`UCLIENT_PROFILE_DISCOVERY`|`OFF`|禁用 DDS 发现协议（由 Agent 代理）|

**资源上限配置**（`rmw_microxrcedds` 部分）：

|选项|FineMote 值|说明|
|---|---|---|
|`RMW_UXRCE_MAX_NODES`|**3**|支持最多 3 个 ROS Node|
|`RMW_UXRCE_MAX_PUBLISHERS`|**10**|最多 10 个 Publisher（5 个电机 \+ 底盘 odom \+ 余量）|
|`RMW_UXRCE_MAX_SUBSCRIPTIONS`|**5**|最多 5 个 Subscriber（cmd\_vel \+ 预留）|
|`RMW_UXRCE_MAX_SERVICES`|1|不使用 ROS 2 Service|
|`RMW_UXRCE_MAX_CLIENTS`|1|不使用 ROS 2 Client|
|`RMW_UXRCE_MAX_HISTORY`|4|Reliable QoS 历史深度（每个 Publisher/Subscriber 最多缓存 4 条消息）|
|`UCLIENT_CUSTOM_TRANSPORT_MTU`|**1024**|自定义传输 MTU（适合 UART 缓冲大小）|
|`UCLIENT_CUSTOM_TRANSPORT_MW_MTU`|**1024**|中间件 MTU（与传输 MTU 一致）|

> **注意**: `RMW_UXRCE_MAX_NODES` 设为 3（而非教程中常见的 1），是因为 FineMote 项目中有多个模块分别注册了独立的 Node。其他数量上限也是根据实际 Agent 数量和消息负载经过权衡确定的——过大会浪费静态内存，过小会导致运行时初始化失败。
> 
> 

#### 构建

```bash
ros2 run micro_ros_setup build_firmware.sh $(pwd)/toolchain.cmake $(pwd)/colcon.meta
```

构建产物：

|产物|路径|说明|
|---|---|---|
|静态库|`microros_ws/firmware/build/libmicroros.a`|交叉编译好的 micro\-ROS 库|
|头文件|`microros_ws/firmware/build/include/`|所有 ROS 2 消息类型定义和 API 头文件|

> **常见错误**: `Could not find ROS middleware implementation 'rmw_microxrcedds'`
> 原因：当前 ROS 2 环境使用的 DDS 中间件不是 Micro\-XRCE\-DDS。
> 解决：`export RMW_IMPLEMENTATION=rmw_microxrcedds` 然后重新构建。
> 
> 

> **头文件嵌套问题**: 某些版本的 `generate_lib` 生成的 include 目录可能存在嵌套结构（如 `include/action_msgs/action_msgs/`），需要手动拍平后复制到项目中。
> 
> 

### 3\.3 将 micro\-ROS 集成到 MCU 项目

将构建好的 micro\-ROS 静态库集成到 MCU 工程中，主要涉及三个层面的工作：**静态库与头文件的 CMake 集成**、**FreeRTOS\-POSIX 适配层配置**，以及 **FreeRTOS 内核参数调整**。这三个层面缺一不可。

#### 3\.3\.1 静态库与头文件的 CMake 集成

以下是 FineMote 项目 `ThirdParty/micro-ROS/CMakeLists.txt` 的实际配置：

```cmake
# ThirdParty/micro-ROS/CMakeLists.txt
cmake_minimum_required(VERSION 3.22)

string(REGEX REPLACE "cortex-(m[0-9]+)" "\\1" MICROROS_VARIANT "${CMAKE_SYSTEM_PROCESSOR}")
set(MICROROS_LIB_FILE "libmicroros_${MICROROS_VARIANT}.a")
set(MICROROS_LIB_PATH "${CMAKE_CURRENT_SOURCE_DIR}/${MICROROS_LIB_FILE}")

add_library(micro-ROS STATIC IMPORTED GLOBAL)

set_property(TARGET micro-ROS PROPERTY IMPORTED_LOCATION ${MICROROS_LIB_PATH})

target_include_directories(micro-ROS INTERFACE
    "${CMAKE_CURRENT_SOURCE_DIR}/include"
)

target_link_libraries(micro-ROS INTERFACE
    FreeRTOS-POSIX
)
```

关键点说明：

|配置项|说明|
|---|---|
|`REGEX REPLACE "cortex-(m[0-9]+)"`|从 `CMAKE_SYSTEM_PROCESSOR`（如 `cortex-m7`）提取 `m7`，自动选择对应的 `.a` 文件|
|`IMPORTED GLOBAL`|声明为导入库（预编译），`GLOBAL` 确保在整个构建中可见|
|`INTERFACE` include/link|头文件路径和依赖通过 `INTERFACE` 传递，不编译任何源文件|
|`FreeRTOS-POSIX` 链接依赖|micro\-ROS 内部依赖 POSIX API（pthread、clock\_gettime 等），由 FreeRTOS\-POSIX 提供|

> 本项目针对不同 Cortex 核心提供了两套预编译库：
> 
> - `libmicroros_m4.a` — Cortex\-M4（STM32L475 等，FPv4\-SP\-D16）
> 
> - `libmicroros_m7.a` — Cortex\-M7（STM32H723/H750 等，FPv5\-SP\-D16）
> 
> 

#### 3\.3\.2 修正头文件目录结构（扁平化处理）

micro\-ROS 静态库构建完成后，`firmware/build/include` 目录中的部分头文件路径可能出现**重复嵌套问题**。例如，期望的 `rcl` 头文件路径为：

```Plain Text
firmware/build/include/
└── rcl/
    ├── allocator.h
    ├── context.h
    └── ...
```

但实际构建结果可能是：

```Plain Text
firmware/build/include/
└── rcl/
    └── rcl/              ← 多余的嵌套！
        ├── allocator.h
        ├── context.h
        └── ...
```

这是因为 ROS 2 包级 include 命名空间在导出静态库时没有被正确扁平化，导致部分路径出现 `pkg/pkg` 形式的重复嵌套。**若不修正该结构，FineMote 在包含 micro\-ROS 头文件时会出现路径不匹配问题，导致编译失败。**

使用以下脚本对 include 目录进行拍平整理：

```bash
#!/bin/bash

LIBRARY_PATH=$(pwd)/firmware/build

pushd firmware/mcu_ws > /dev/null
INCLUDE_ROS2_PACKAGES=$(colcon list | awk '{print $1}')
popd > /dev/null

for var in ${INCLUDE_ROS2_PACKAGES}; do
    if [ -d "$LIBRARY_PATH/include/${var}/${var}" ]; then
        rsync -r "$LIBRARY_PATH/include/${var}/${var}/" "$LIBRARY_PATH/include/${var}/"
        rm -rf "$LIBRARY_PATH/include/${var}/${var}"
    fi
done
```

> 该脚本遍历 colcon 工作区中所有包，检测是否存在 `pkg/pkg/` 形式的嵌套目录，若存在则将其内容上移一层并删除多余嵌套。这是 micro\-ROS 构建流程中的一个已知问题，参见 [micro\_ros\_setup Issue \#530](https://github.com/micro-ROS/micro_ros_setup/issues/530)。
> 
> 

#### 3\.3\.3 配置 FreeRTOS\-POSIX 依赖

**FreeRTOS 与 POSIX 的关系**

要理解为什么需要 FreeRTOS\-POSIX，首先要厘清两个概念：

||FreeRTOS|POSIX|
|---|---|---|
|**本质**|实时操作系统内核|操作系统接口标准（IEEE Std 1003\.1）|
|**API 风格**|`xTaskCreate()`, `xQueueSend()`, `vTaskDelay()`|`pthread_create()`, `clock_gettime()`, `sem_wait()`|
|**设计目标**|资源受限 MCU 的实时调度|应用程序在 Unix/Linux 之间的可移植性|
|**典型环境**|STM32, ESP32 等嵌入式平台|Linux, macOS, Unix|

这两套 API **互不认识**——FreeRTOS 不理解 `pthread_create`，POSIX 也不理解 `xTaskCreate`。

**FreeRTOS\-POSIX 的角色：翻译官**

micro\-ROS 最初为 POSIX 系统（Linux）设计，其内部代码大量使用 `pthread`、`clock_gettime`、`semaphore` 等 POSIX 调用。为了让 micro\-ROS 在 FreeRTOS 上运行，需要一个"翻译层"将 POSIX 调用映射到 FreeRTOS 的等效操作：

```Plain Text
micro-ROS 代码                        FreeRTOS 内核
     │                                      │
     │ pthread_create(...)                   │
     │ clock_gettime(...)                    │
     │ pthread_mutex_lock(...)               │
     │                                      │
     └──────► FreeRTOS-POSIX ◄──────────────┘
              (翻译层 / 适配层)

    pthread_create()    →  xTaskCreate()
    clock_gettime()     →  vTaskSetTimeOutState() + tick 计数
    pthread_mutex_lock() →  xSemaphoreTake()
    sem_wait()          →  xSemaphoreTake()
    usleep()            →  vTaskDelay()
```

FreeRTOS\-POSIX（Amazon 官方命名为 FreeRTOS\+POSIX）由 AWS 开发，以 MIT 许可证开源。它**不是**在 FreeRTOS 上跑一个 Linux 子系统，而是提供了一层薄薄的 API 映射——每个 POSIX 函数内部直接调用等价的 FreeRTOS API。

以下是 FineMote 项目 `ThirdParty/CMakeLists.txt` 中 FreeRTOS\-POSIX 的实际配置：

```cmake
# ThirdParty/CMakeLists.txt (FreeRTOS-POSIX 部分)
add_library(FreeRTOS-POSIX STATIC)

target_include_directories(FreeRTOS-POSIX PUBLIC
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/include/private"
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include"
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable"
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable/st/stm32l475_discovery"
)

file(GLOB FreeRTOS_POSIX_SOURCES CONFIGURE_DEPENDS
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/*.c"
)

target_sources(FreeRTOS-POSIX PRIVATE ${FreeRTOS_POSIX_SOURCES})
target_link_libraries(FreeRTOS-POSIX PRIVATE finemote::board)
```

五个 include 路径的职责：

|路径|内容|
|---|---|
|`include/`|公共 POSIX 头文件封装（`FreeRTOS_POSIX.h` 等）|
|`include/private/`|内部使用的辅助宏和类型|
|`FreeRTOS-Plus-POSIX/include/`|POSIX 标准头文件（`pthread.h`, `time.h`, `mqueue.h` 等）|
|`portable/`|平台默认配置（`FreeRTOS_POSIX_portable_default.h`）|
|`portable/st/stm32l475_discovery/`|**STM32 平台移植头**（见下方说明）|

##### ⚠️ 关键：`st/stm32l475_discovery` 路径

最后一个路径最容易被遗漏：

```Plain Text
FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable/st/stm32l475_discovery
```

这个目录下只有一个文件 `FreeRTOS_POSIX_portable.h`，内容仅两行有效代码：

```c
/* This port uses the defaults in FreeRTOS_POSIX_portable_default.h,
 * so this file is empty. */
```

STM32 系列全部沿用默认配置，所以这个文件本身是空的。FreeRTOS\-Plus\-POSIX 为不同平台提供了多个移植目录：

|移植路径|目标平台|
|---|---|
|`portable/st/stm32l475_discovery`|ST STM32L475 Discovery 板（**FineMote 使用**）|
|`portable/st/stm32h745zi_nucleo`|ST STM32H745 Nucleo 板|
|`portable/ti/cc3220_launchpad`|TI CC3220 LaunchPad|
|`portable/nxp/lpc54018iotmodule`|NXP LPC54018 IoT Module|
|`portable/pc/windows`|Windows 模拟器|

> **重要**: 即使 MCU 是 STM32H7 而非 STM32L4，仍然使用 `stm32l475_discovery` 这个路径——这是 FreeRTOS\-Plus\-POSIX 中 ST 官方支持的唯一 STM32 移植，且各 STM32 系列的 POSIX 移植定义完全一致（均沿用默认配置）。
> 
> 

> **⚠️ 若缺失此路径**: 编译阶段不会报错，但运行时可能出现异常行为。这是因为编译器在找不到指定平台移植头时不会中止编译（取决于工具链配置和警告级别），但 FreeRTOS\-POSIX 内部的部分平台相关类型定义和宏可能未被正确解析，导致运行时的隐式行为差异。因此配置时必须确保此路径与实际目录结构一致。
> 
> 

#### 3\.3\.4 修改 FreeRTOS 配置

此外，需要在 `FreeRTOSConfig.h` 中启用以下宏定义，以满足 FreeRTOS\-POSIX 的运行需求：

```c
/* USER CODE BEGIN Defines */
#define configUSE_POSIX_ERRNO          1
#define configUSE_APPLICATION_TASK_TAG 1
/* USER CODE END Defines */
```

|宏|作用|
|---|---|
|`configUSE_POSIX_ERRNO`|启用 POSIX 兼容的 `errno` 支持，FreeRTOS\-POSIX 的错误处理依赖此宏|
|`configUSE_APPLICATION_TASK_TAG`|启用任务标签功能，FreeRTOS\-POSIX 内部使用 `pvTaskGetThreadLocalStoragePointer` 等 API 需要此支持|

> **注意**: 这两个宏必须定义在 `FreeRTOSConfig.h` 的 `USER CODE` 区域内（如果使用 CubeMX 生成代码），否则重新生成代码时会被覆盖。
> 
> 

### 3\.4 创建移植层（传输层 \+ 时钟 \+ 分配器）

在完成库集成和 POSIX 适配后，还需实现 micro\-ROS 所需的底层接口：**传输层函数**、**时钟函数**和**内存分配器**。

#### 3\.4\.1 传输层函数

micro\-ROS 自定义传输层需要实现 4 个函数，并通过 `rmw_uros_set_custom_transport()` 注册：

|函数|职责|关键要求|
|---|---|---|
|`open()`|初始化通信外设|如外设已初始化，可为空（返回 `true`）|
|`close()`|关闭通信外设|通常可为空（返回 `true`）|
|`write(buf, len)`|发送数据|必须至少发送 1 字节，返回实际发送字节数；失败返回 0|
|`read(buf, len, timeout)`|接收数据|超时时间内至少读 1 字节，超时则设错误标志并返回 0|

注册方式（FineMote 实际代码，位于 `MicroROS_Manager.hpp` 构造函数中）：

```cpp
rmw_uros_set_custom_transport(
    true,                           // 启用 stream framing（串口通信必需）
    nullptr,                        // 自定义参数（未使用）
    MicroROS_Transport::Open,       // open 函数
    MicroROS_Transport::Close,      // close 函数
    MicroROS_Transport::Write,      // write 函数
    MicroROS_Transport::Read        // read 函数
);
```

> **说明**: `rmw_uros_set_custom_transport` 必须在 `rclc_support_init()` 之前调用。FineMote 将此调用放在 `MicroROS_Manager` 的构造函数中，确保在状态机启动前完成注册。
> 
> 

#### 3\.4\.2 时钟函数 \(`clock_gettime`\)

micro\-ROS 内部需要获取系统时间用于超时计算、消息时间戳等。所需的时间精度为毫秒级（ms），函数签名：

```c
int clock_gettime(clockid_t clock_id, struct timespec *tp);
// tp->tv_sec:  秒
// tp->tv_nsec: 纳秒
```

**FineMote 项目的实际做法**: `clock_gettime`**不需要用户自己实现**。它由 FreeRTOS\-POSIX 适配层提供（`ThirdParty/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/source/FreeRTOS_POSIX_clock.c`），基于 FreeRTOS 的 tick 计数器实现：

- 通过 `vTaskSetTimeOutState()` 获取 tick 计数和溢出次数

- 将 tick 数乘以 `NANOSECONDS_PER_TICK` 转换为纳秒

- 使用 `UTILS_NanosecondsToTimespec()` 填充 `tv_sec` 和 `tv_nsec`

```c
// FreeRTOS-POSIX 内部实现（简化）
int clock_gettime(clockid_t clock_id, struct timespec *tp) {
    TimeOut_t xCurrentTime;
    vTaskSetTimeOutState(&xCurrentTime);
    uint64_t ullTickCount =
        (uint64_t)xCurrentTime.xOverflowCount << (sizeof(TickType_t) * 8);
    ullTickCount += xCurrentTime.xTimeOnEntering;
    UTILS_NanosecondsToTimespec(
        (int64_t)ullTickCount * NANOSECONDS_PER_TICK, tp);
    return 0;
}
```

> **关键**: FreeRTOS\-POSIX 提供的 `clock_gettime` 分辨率等于 FreeRTOS 的 tick 周期（通常为 1ms）。在 colcon\.meta 构建选项中设置 `CLOCK_MONOTONIC=0` 是因为 ARM 工具链的 `<time.h>` 缺少该宏定义，而不是因为不需要时钟。
> 
> 

#### 3\.4\.3 内存分配器

micro\-ROS 内部使用 `rcl_allocator_t` 结构体来管理动态内存分配，默认情况下使用标准 C 库的 `malloc`/`free`。FineMote 项目直接使用**默认分配器**：

```cpp
// MicroROS_Manager 构造函数中（实际代码）
allocator_ = rcl_get_default_allocator();
```

默认分配器直接调用 C 标准库的 `malloc`/`free`/`calloc`/`realloc`。在 FreeRTOS 环境中，这些函数由 FreeRTOS 的堆实现提供（如 `heap_4.c`），底层实际调用 `pvPortMalloc`/`vPortFree`，因此已经具备线程安全性。

> **备选方案 —— 显式自定义分配器**: 如果希望显式控制分配器（例如使用静态内存池），可以在 `rclc_support_init()` 之前设置：
> 
> ```c
> static void * custom_allocate(size_t size, void * state) {
>     return pvPortMalloc(size);   // FreeRTOS 线程安全分配
> }
> static void custom_deallocate(void * pointer, void * state) {
>     vPortFree(pointer);
> }
> // ... reallocate, zero_allocate 同理 ...
> 
> rcl_allocator_t allocator = rcutils_get_zero_initialized_allocator();
> allocator.allocate = custom_allocate;
> allocator.deallocate = custom_deallocate;
> rcutils_set_default_allocator(&allocator);
> ```
> 
> FineMote 不采用此方式，因为 FreeRTOS 的 `heap_4.c` 已经将 `malloc`/`free` 映射到 `pvPortMalloc`/`vPortFree`，无需额外包装。
> 
> 

#### 3\.4\.4 小结：各接口提供者

|接口|提供者|实现位置|
|---|---|---|
|`open/close/write/read`|用户实现|`Services/MicroROS/MicroROS_Transport.hpp`|
|`clock_gettime`|**FreeRTOS\-POSIX**（无需用户实现）|`ThirdParty/FreeRTOS-POSIX/.../FreeRTOS_POSIX_clock.c`|
|内存分配器|**默认分配器**（`rcl_get_default_allocator()`）|FreeRTOS `heap_4.c` → `pvPortMalloc`/`vPortFree`|

---

## 4\. FineMote micro\-ROS 实现

本章节以上述 micro\-ROS 构建与移植为基础，介绍 FineMote 项目中的工程化封装实现。

### 4\.1 整体架构

FineMote 将 micro\-ROS 封装为 Service 层，包含 4 个核心文件：

```Plain Text
Services/MicroROS/
├── MicroROS_Manager.hpp     # 核心管理器（状态机 + FreeRTOS 线程）
├── MicroROS_Agent.hpp       # Publisher/Subscriber 模板类
├── MicroROS_Transport.hpp   # 自定义 UART 传输层
└── MicroROS_MessageTypes.hpp # 消息类型注册
```

**依赖关系**:

```Plain Text
ThirdParty/micro-ROS/         # 预编译静态库 + 头文件
    ├── libmicroros_m7.a      # Cortex-M7 库
    ├── libmicroros_m4.a      # Cortex-M4 库
    └── include/              # ROS 2 消息头文件

ThirdParty/FreeRTOS-POSIX/    # FreeRTOS 的 POSIX 兼容层
    └── (提供 pthread, clock_gettime 等 POSIX API)

BSP/                          # 板级支持包
    └── (提供 UART HAL、DMA 配置等)
```

**设计原则**:

- **编译期开关**: Manager 模板参数 `bool enable` 支持编译期启用/禁用（`static_assert` 在禁用时报错）

- **去 FreeRTOS 耦合**: 传输层不依赖 FreeRTOS（`c1e578c`, `b304be7`），仅使用 C\+\+ 标准库 `std::atomic`

- **单例模式**: Manager 使用单例，全局唯一入口

### 4\.2 通信概念：Publisher、Subscriber 与 QoS

在深入具体实现之前，先理清 micro\-ROS 中最基础的三个通信概念。

#### Publisher（发布者）

Publisher 是消息的**生产者**。MCU 将传感器数据、状态信息等封装为 ROS 2 消息，通过指定的话题（Topic）发布到 ROS 2 网络中。任何订阅了该话题的节点（包括 PC 端的 ROS 2 节点）都能收到这些消息。

在 micro\-ROS 底层，Publisher 通过 `rcl_publish()` 将消息交给 DDS 中间件，中间件负责序列化并通过传输层发送到 Agent，再由 Agent 转发到 ROS 2 网络。

#### Subscriber（订阅者）

Subscriber 是消息的**消费者**。MCU 订阅某个话题后，每当有新消息到达，注册的回调函数会被执行器（Executor）调用。典型的应用场景是接收 PC 端下发的控制指令（如 `cmd_vel` 速度指令）。

回调函数在执行器的 `spin` 循环中被调用，因此回调中不应执行长时间阻塞操作。

#### QoS（服务质量）

micro\-ROS 支持两种主要的 QoS 模式，通过不同的 `rclc_publisher_init_*` / `rclc_subscription_init_*` 函数选择：

|特性|Reliable（默认）|Best\-Effort|
|---|---|---|
|消息保证送达|✅ 是|❌ 不保证|
|重传机制|✅ 有|❌ 无|
|资源消耗|较高（需维护历史缓存）|较低|
|吞吐量和延迟|较低|较高|
|适用场景|关键状态/指令（如 `cmd_vel`）|高频传感器数据（如 `joint_states`）|

> **选择建议**: Reliable 模式依赖 `RMW_UXRCE_MAX_HISTORY` 配置的历史缓冲区。FineMote 中 Reliable 设为默认值 4，意味着最多缓存 4 条未确认消息。若 Publisher 发布频率过高（\> Agent 转发速度），历史缓冲会填满并阻塞 `rcl_publish()`。对于高频话题（\> 100Hz），应优先使用 Best\-Effort。
> 
> 

#### 连接建立过程

MCU 开机后，`rclc_support_init()` 会尝试与 Agent 建立连接。底层 Micro\-XRCE\-DDS 客户端发送 XRCE CREATE 消息（包含 "XRCE" 标识符），默认重试 10 次、每次间隔 1 秒。如果 10 次都未收到 Agent 响应，`rclc_support_init()` 返回超时错误。FineMote 通过状态机中的 `WAITING_AGENT` 状态循环等待，支持无限重试。

### 4\.3 自定义 UART 传输层

#### 设计特点

|特性|实现|
|---|---|
|物理层|STM32 UART \+ DMA|
|缓冲区|2048 字节环形缓冲区（RX），位掩码取模|
|并发控制|`std::atomic` 无锁操作（`rx_w_` / `rx_r_`）|
|TX 重试|`tx_busy_` 忙标志 \+ CAS 锁 \+ 100ms 超时（100 × 1ms）|
|RX 超时|轮询间隔 5ms（`usleep(5000)`）|
|帧协议|micro\-ROS 标准 stream framing|

#### 关键实现

```cpp
// MicroROS_Transport.hpp（实际代码，精简）
class MicroROS_Transport {
public:
    static MicroROS_Transport& GetInstance();  // 单例

    // ──── 四个传输函数（静态方法，符合 micro-ROS 自定义传输接口）────
    static bool Open(struct uxrCustomTransport* t);
    static bool Close(struct uxrCustomTransport* t);
    static size_t Write(struct uxrCustomTransport* t,
                        const uint8_t* buf, size_t len, uint8_t* err);
    static size_t Read(struct uxrCustomTransport* t,
                       uint8_t* buf, size_t len, int timeout, uint8_t* err);

private:
    // 环形缓冲区（2 的幂大小，利用位掩码取模）
    static constexpr size_t MICROROS_BUF_SIZE = 2048;
    uint8_t rx_buf_[MICROROS_BUF_SIZE];
    std::atomic<size_t> rx_w_{0};   // write index
    std::atomic<size_t> rx_r_{0};   // read index

    // DMA 接收缓冲区
    static constexpr size_t MICROROS_DMA_BUF_SIZE = 512;
    UARTBuffer<MICRO_ROS_UART_ID, MICROROS_DMA_BUF_SIZE> dma_buffer_;

    // TX 忙标志（无锁 CAS）
    std::atomic<bool> tx_busy_{false};
    uint8_t tx_buffer_[MICROROS_BUF_SIZE];
};
```

#### 为什么使用自定义 UART 传输而不是默认 Serial？

1. **DMA 支持**: 默认 Serial 传输使用中断模式，高波特率下 CPU 占用高

2. **精细控制**: 可以控制超时、重试策略等底层行为

3. **多板卡适配**: 不同板卡的 UART 引脚和 DMA 通道不同

4. **与 BSP 层集成**: 复用项目已有的 UART HAL 封装

### 4\.4 状态机管理

MicroROS Manager 使用四状态机模型，运行在独立的 FreeRTOS 线程中。

```Plain Text
┌──────────────┐
                 │ WAITING_AGENT│ 等待 Agent 连接
                 └──────┬───────┘
                        │ Agent 握手成功
                        ▼
                 ┌──────────────┐
          ┌──────│ INITIALIZING │ 初始化所有 Agent
          │      └──────┬───────┘
          │             │ 全部 Agent Init() 成功
          │             ▼
          │      ┌──────────────┐
          │      │   RUNNING    │ 正常运行（spin + ping）
          │      └──────┬───────┘
          │             │ 异常（ping 超时 / spin 出错）
          │             ▼
          │      ┌──────────────┐
          └──────│    ERROR     │ 错误状态，待重连
                 └──────────────┘
```

状态流转逻辑：

- **WAITING\_AGENT → INITIALIZING**: `rmw_uros_ping_agent()` 成功（Agent 握手完成，随后在 INITIALIZING 中调用 `rclc_support_init()`）

- **INITIALIZING → RUNNING**: 所有已注册 Agent 的 `Init()` 全部成功

- **INITIALIZING → ERROR**: 任一 Agent `Init()` 失败

- **RUNNING → ERROR**: `rclc_executor_spin_some()` 返回错误 或 ping 超时

- **ERROR → WAITING\_AGENT**: 延迟 1 秒后自动跳转，重新等待连接

#### 关键配置参数

|参数|默认值|说明|
|---|---|---|
|`THREAD_LOOP_INTERVAL_MS`|200ms|Manager 线程循环间隔|
|`PING_INTERVAL_CYCLES`|5|每 5 次循环 ping 一次（即每 1 秒）|
|`PING_TIMEOUT_MS`|500ms|Ping 超时时间|
|`EXECUTOR_SPIN_TIMEOUT_MS`|10ms|执行器 spin 超时|
|`STACK_SIZE`|4096 bytes|Manager 线程栈大小|
|`MAX_HANDLES`|10|执行器最大句柄数|
|`MAX_AGENTS`|10|最大 Agent 数量|

#### 栈监控

由于 micro\-ROS 内部调用栈深度大，**栈溢出是最常见的运行时问题之一**。许多看似与通信协议或 Agent 握手相关的异常（如连接后立即断开、HardFault、随机卡死），根本原因往往是栈空间不足——尤其是 `rclc_executor_spin_some()` 处理新消息时的调用栈深度远超一般 RTOS 任务。排查 micro\-ROS 运行问题时，建议**首先检查栈大小是否足够**。

Manager 内建了栈高水位监控：通过 FreeRTOS 的 `uxTaskGetStackHighWaterMark()` 在每次 `HandleRunning()` 周期中记录 Manager 线程自身的栈剩余，同时对比各阶段（spin、初始化、各 Agent Execute）的栈变化 delta，汇总为 `StackInfo` 结构体供外部查询。

> **实测经验**: 本项目通过栈高水位监控实测了各阶段的栈使用峰值，经过多轮压测后确定了以下可运行的最小值（commit `c6abd56`）：
> 
> 

|参数|实测值|说明|
|---|---|---|
|Manager 线程栈|`4 * 1024`（4096 bytes）|从初始的 `20 * 1024`（20480 bytes）大幅缩减|
|FreeRTOS 堆（M7 板卡）|`configTOTAL_HEAP_SIZE` = 20480 bytes|MC\_Board、MC\_Board\_02|
|FreeRTOS 堆（M4 板卡）|`configTOTAL_HEAP_SIZE` = 16384 bytes|Robomaster\_C|
|FreeRTOS 最小任务栈|`configMINIMAL_STACK_SIZE` = 512 words（2 KB）|所有板卡统一|

这些值是通过 `uxTaskGetStackHighWaterMark()` 实测后确定的较紧凑配置——增大不会提升性能，但减小可能导致栈溢出。连接稳定性方面，ping 超时从 100ms 增至 500ms，Publisher 初始化模式从 best\-effort 改为 default（reliable）。

### 4\.5 Agent 模型（Publisher/Subscriber 模板）

#### 基类设计

```cpp
// MicroROS_Agent.hpp（实际代码）
class ROSAgent {
public:
    ROSAgent()
    {
        MicroROS_Manager<>::GetInstance().RegisterAgent(this);
    }

    virtual bool Init(rcl_node_t* node, rclc_support_t* support,
                      rclc_executor_t* executor) = 0;
    virtual void Execute() = 0;
    virtual void Fini() = 0;

    /// 供栈监控/调试输出的可读标签
    virtual const char* GetTopicName() const { return "unknown"; }
};
```

所有通信实体（Publisher、Subscriber）继承自 `ROSAgent`。构造函数中自动向 Manager 注册，无需手动调用 `RegisterAgent()`。Manager 通过统一的 `Init()` / `Execute()` / `Fini()` 接口批量管理，无需区分具体类型。

#### RosPublisher\<MsgT\> 模板

```cpp
// MicroROS_Agent.hpp（实际代码，精简）
template <typename MsgT>
class RosPublisher : public ROSAgent<> {
    static_assert(RosMsgTraits<MsgT>::registered,
        "RosPublisher<MsgT>: MsgT is not registered.");

    using ConverterFunc = std::function<void(MsgT&)>;

    // 重载 1：对象有 GetRosBinder() 方法（如 JointState 等复杂消息）
    template <typename ObjT>
    RosPublisher(const char* obj_name, ObjT& obj) :
        converter_(obj.GetRosBinder()),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + obj_name
                   + "/" + RosMsgTraits<MsgT>::name) {}

    // 重载 2：直接传入转换函数
    template <typename FuncT>
    RosPublisher(const char* base_name, FuncT&& func) :
        converter_(std::forward<FuncT>(func)),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name
                   + "/" + RosMsgTraits<MsgT>::name) {}

    // Init: 用 rclc_publisher_init_default（Reliable QoS）
    bool Init(rcl_node_t* node, rclc_support_t*, rclc_executor_t*) final;
    void Execute() final { converter_(msg_); rcl_publish(&publisher_, &msg_, nullptr); }
    void Fini() final { rcl_publisher_fini(&publisher_, nullptr); }

    std::string topic_str_;
    ConverterFunc converter_;
    rcl_publisher_t publisher_{};
    MsgT msg_{};
};
```

关键设计点：

- 消息存储在 Publisher **内部**（`msg_` 成员），由 `ConverterFunc` 在每次 `Execute()` 时填充最新数据后发布

- 构造函数根据传入对象是否有 `GetRosBinder()` 方法做 SFINAE 重载，自动拼接 topic 字符串

- `Init()` 使用 `rclc_publisher_init_default()` → **Reliable QoS**

#### RosSubscriber\<MsgT\> 模板

```cpp
// MicroROS_Agent.hpp（实际代码，精简）
template <typename MsgT>
class RosSubscriber : public ROSAgent<> {
    static_assert(RosMsgTraits<MsgT>::registered,
        "RosSubscriber<MsgT>: MsgT is not registered.");

    using CallbackFunc = std::function<void(const MsgT&)>;

    // 唯一构造函数：base_name + 回调函数
    template <typename FuncT>
    RosSubscriber(const char* base_name, FuncT&& callback) :
        callback_(std::forward<FuncT>(callback)),
        topic_str_(std::string(MICROROS_NODE_NAME) + "/" + base_name) {}

    // Init: 使用 rclc_subscription_init_best_effort（Best-Effort QoS）
    bool Init(rcl_node_t* node, rclc_support_t*, rclc_executor_t* executor) final;
    void Execute() final {}   // Subscriber 的回调在 spin 中触发，此处为空
    void Fini() final { rcl_subscription_fini(&subscriber_, nullptr); }

    std::string topic_str_;
    CallbackFunc callback_;
    rcl_subscription_t subscriber_{};
    MsgT msg_{};
};
```

关键设计点：

- Topic 格式为 `{MICROROS_NODE_NAME}/{base_name}`，**不带消息类型名**（与 Publisher 不同）

- `Init()` 使用 `rclc_subscription_init_best_effort()` → **Best\-Effort QoS**（高频话题优先吞吐）

- `Execute()` 为空：Subscriber 的回调由执行器的 `spin_some()` 在 `HandleRunning()` 中统一触发

- 消息类型由回调函数签名自动推导（`callback_message_type` trait）

#### Topic 命名规则

|类型|命名格式|示例|
|---|---|---|
|Publisher|`{MICROROS_NODE_NAME}/{obj_name}/{MsgName}`|`FineMote/chassis_odom/Odometry`|
|Subscriber|`{MICROROS_NODE_NAME}/{base_name}`|`FineMote/cmd_vel`|

> ⚠️ **命名大小写**: ROS 2 的 topic 名称区分大小写——`/FineMote/cmd_vel` 和 `/finemote/cmd_vel` 是两个不同的话题，不会互通。`MICROROS_NODE_NAME` 在 `MicroROS_Agent.hpp` 中定义为 `"FineMote"`（大写 F、大写 M），所有 topic 均以此命名空间为前缀。PC 端订阅或发布话题时必须使用完全相同的名称。
> 
> 

#### CTAD 推导指南

```cpp
// Publisher: 根据 converter 函数签名推导 MsgT
template <typename ObjT>
RosPublisher(const char*, ObjT&)
    -> RosPublisher<callback_message_type_t<decltype(std::declval<ObjT&>().GetRosBinder())>>;

template <typename FuncT>
RosPublisher(const char*, FuncT&&)
    -> RosPublisher<callback_message_type_t<FuncT>>;

// Subscriber: 根据回调函数签名推导 MsgT
template <typename FuncT>
RosSubscriber(const char*, FuncT&&)
    -> RosSubscriber<callback_message_type_t<FuncT>>;
```

### 4\.6 消息类型注册机制

#### 宏定义注册

```cpp
// MicroROS_MessageTypes.hpp（实际代码）
template <typename T>
struct RosMsgTraits {
    static constexpr bool registered = false;  // 默认：未注册
};

// 4 参数宏：C++ 类型, ROS 包名, 子目录, 消息名
#define DEFINE_MICROROS_MSG(CppType, PkgName, MsgSub, MsgName) \
    template <> \
    struct RosMsgTraits<CppType> { \
        static constexpr bool registered = true; \
        static constexpr const char* name = #MsgName; \
        static const rosidl_message_type_support_t* GetTypeSupport() { \
            return ROSIDL_GET_MSG_TYPE_SUPPORT(PkgName, MsgSub, MsgName); \
        } \
    };

// 便捷宏：根据对象名自动生成 RosPublisher
#define PUBLISHER(obj) RosPublisher(#obj, obj)
```

实际注册示例：

```cpp
DEFINE_MICROROS_MSG(sensor_msgs__msg__JointState, sensor_msgs, msg, JointState)
DEFINE_MICROROS_MSG(geometry_msgs__msg__Twist,      geometry_msgs, msg, Twist)
DEFINE_MICROROS_MSG(std_msgs__msg__Bool,             std_msgs, msg, Bool)
DEFINE_MICROROS_MSG(std_msgs__msg__Int32,            std_msgs, msg, Int32)
DEFINE_MICROROS_MSG(std_msgs__msg__String,           std_msgs, msg, String)
```

#### 已注册的消息类型

|类型|ROS 2 消息|用途|
|---|---|---|
|`JointState`|`sensor_msgs/msg/JointState`|电机关节状态发布|
|`Twist`|`geometry_msgs/msg/Twist`|底盘速度指令订阅|
|`Bool`|`std_msgs/msg/Bool`|布尔状态发布/订阅|
|`Int32`|`std_msgs/msg/Int32`|整数数据|
|`String`|`std_msgs/msg/String`|字符串数据|

#### 消息类型支持检查

```cpp
// MicroROS_Agent.hpp（实际代码）
// 编译期 trait：检测类型是否有 GetRosBinder 方法
template <typename T, typename = void>
struct has_GetRosBinder : std::false_type {};

template <typename T>
struct has_GetRosBinder<T,
    std::void_t<decltype(std::declval<T&>().GetRosBinder())>>
    : std::true_type {};

template <typename T>
inline constexpr bool has_GetRosBinder_v = has_GetRosBinder<T>::value;
```

此 trait 用于 RosPublisher 的构造函数 SFINAE 重载选择：有 `GetRosBinder()` 的对象使用第一个重载，否则使用第二个重载。

### 4\.7 StateSnapshot 机制

StateSnapshot 是一个双缓冲、原子操作的状态容器，用于在中断/线程间安全传递电机和底盘状态数据。

```cpp
template <typename T>
class StateSnapshot {
    // 双缓冲：一个当前值、一个待发布值
    // 原子交换：通过指针交换实现无锁读写

    // 写端（数据生产者，如 CAN 回调）
    T &GetWriteBuffer();

    // 读端（数据消费者，如 ROS Publisher）
    const T &GetReadBuffer() const;
};
```

#### 使用场景

```Plain Text
CAN 中断回调                          ROS 发布线程
    │                                      │
    ▼                                      ▼
MotorBase::Update()                  MicroROS Manager::Execute()
    │                                      │
    ▼                                      ▼
snapshot_.GetWriteBuffer()          snapshot_.GetReadBuffer()
    │                                      │
    ▼                                      ▼
写入最新电机状态                    发布 sensor_msgs/JointState
    │                                      │
    ▼                                      ▼
    └──── atomic swap ────────────────────┘
```

### 4\.8 板级支持

#### 支持的开发板

|板卡|MCU|核心|编译预设|
|---|---|---|---|
|MC\_Board|STM32H750|Cortex\-M7|`MC_Board`|
|MC\_Board\_02|STM32H723|Cortex\-M7|`MC_Board_02`|
|Robomaster\_C|STM32F407|Cortex\-M4|`Robomaster_C`|
|Robomaster\_A|STM32F427|Cortex\-M4|`Robomaster_A`|

#### BSP 配置（以 MC\_Board 为例）

```cpp
// ProjectConfig.hpp (各板卡)
#define WITH_MICRO_ROS           // 启用 micro-ROS 功能
#define MICRO_ROS_UART_ID    6   // 使用 UART6 作为 micro-ROS 通信接口
```

> **注意**: `WITH_MICRO_ROS` 宏定义是编译期开关，未定义则 Manager 模板在运行时 `static_assert` 报错。
> 
> 

#### FreeRTOS 配置要求

```cpp
// FreeRTOSConfig.h
#define configTOTAL_HEAP_SIZE    // 堆大小（micro-ROS 需要较大堆）
#define configMINIMAL_STACK_SIZE 512  // 最小任务栈（512 words）
#define configUSE_POSIX          // POSIX 兼容层
```

#### 相关电机驱动

|驱动|电机型号|StateSnapshot 支持|
|---|---|---|
|Emm28|步进电机|✅|
|HO3507|直流无刷|✅|
|Motor4010|直流无刷|✅|
|Motor4315|直流无刷|✅|
|Odrive|ODrive 驱动板|✅|

---

## 5\. 搭建与使用

### 5\.1 上位机（Host PC）配置

参考 [Not Black Magic 教程](https://notblackmagic.com/bitsnpieces/micro-ros/) 和已有飞书文档 [micro\-ROS上位机配置步骤](https://my.feishu.cn/docx/T45fdeQb8o43qVxA3sncWSwWnVg)。

#### 首次配置

```bash
# 1. 安装 ROS 2 Humble（Ubuntu 22.04）
# 参考: https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html

# 2. 安装依赖
sudo apt install python3-colcon-common-extensions python3-pip

# 3. 创建 micro-ROS 工作区
mkdir -p ~/microros_ws/src
cd ~/microros_ws
git clone -b humble https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
rosdep update && rosdep install --from-paths src --ignore-src -y
colcon build
source install/local_setup.bash

# 4. 构建 Agent
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
source install/local_setup.bash
```

#### 构建新静态库（如需修改消息类型或资源上限）

```bash
# 1. 创建固件构建环境
ros2 run micro_ros_setup create_firmware_ws.sh generate_lib

# 2. 编写或修改 toolchain.cmake 和 colcon.meta

# 3. 构建
ros2 run micro_ros_setup build_firmware.sh $(pwd)/toolchain.cmake $(pwd)/colcon.meta

# 4. 将生成的 libmicroros.a 复制到项目 ThirdParty/micro-ROS/
#    将 include/ 目录复制到项目 ThirdParty/micro-ROS/include/
```

### 5\.2 下位机（MCU）配置

#### CMake 构建

```bash
# 使用 CMakePresets 按板卡选择
cmake --preset MC_Board_02
cmake --build --preset MC_Board_02
```

#### 在代码中添加 micro\-ROS Agent

```cpp
// 1. 在 MicroROS_MessageTypes.hpp 中注册新消息类型
DEFINE_MICROROS_MSG(MyNewMsg)

// 2. 在 Task 文件中创建 Agent
auto pub = PUBLISHER(my_object);  // 自动创建 RosPublisher

// 3. Manager 自动管理 Agent 的生命周期
MicroROSManager::Instance().RegisterAgent(&pub);
```

### 5\.3 日常使用流程

```bash
# ---------- PC 端 ----------
cd ~/microros_ws
source install/local_setup.bash

# 启动 micro-ROS Agent（连接到 STM32 串口）
ros2 run micro_ros_agent micro_ros_agent serial -b 921600 --dev /dev/ttyACM2 -v6

# 查看话题列表
ros2 topic list

# 查看话题内容
ros2 topic echo /FineMote/joint_states

# 发布控制指令
ros2 topic pub /FineMote/cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.5}, angular: {z: -1.0}}"

# ---------- MCU 端 ----------
# 1. 烧录固件到 STM32
# 2. 上电 / 复位
# 3. MCU 上电后自动连接 Agent，LED 指示连接状态
```

---

## 6\. 注意事项与踩坑记录

### 栈溢出问题

**症状**: MCU 运行时卡死或 HardFault，且在连接建立后随机出现（而非每次都复现）。

**原因**: micro\-ROS 内部函数调用栈深度大，尤其 `rclc_executor_spin_some()` 处理新消息时需要较大的栈空间。许多看似与通信协议或 Agent 握手相关的异常，根本原因往往是栈不足。

**经过实测验证的参数**（commit `c6abd56`）:

|参数|M7 板卡 \(MC\_Board / MC\_Board\_02\)|M4 板卡 \(Robomaster\_C\)|
|---|---|---|
|Manager 线程栈|4096 bytes|4096 bytes|
|`configTOTAL_HEAP_SIZE`|20480 bytes|16384 bytes|
|`configMINIMAL_STACK_SIZE`|512 words|512 words|

**诊断方法**:

- 启用 `configCHECK_FOR_STACK_OVERFLOW` 进行诊断

- 使用 `uxTaskGetStackHighWaterMark()` 监控实际栈用量

- 怀疑栈问题时，可先将栈/堆值翻倍测试，排除后再逐步缩减

### 编译选项兼容性

**`-fshort-wchar`**** 问题**（commit `56f140c` / `738d92c`）:

ARM GCC 默认 `wchar_t` 为 32 位（4 字节），而 `-fshort-wchar` 将其改为 16 位（2 字节，与 ARM Compiler / MSVC 一致）。micro\-ROS 库内部使用 `wchar_t` 类型，如果静态库编译时和 MCU 项目编译时的 `wchar_t` 宽度不一致，会导致链接符号不匹配和运行时 ABI 错误。

**FineMote 的策略**: 静态库和项目代码**统一启用**`-fshort-wchar`（两侧一致），在 `toolchain.cmake` 中通过 `OPT_FLAGS` 统一注入此标志。项目代码的 CMake 配置也需要添加此标志以保持一致。

> 本项目历史上曾经历 `-fshort-wchar` 反复调整：commit `56f140c` 首次启用，commit `738d92c` 曾短暂禁用以排查兼容性问题，最终确认统一启用策略。
> 
> 

**`-DCLOCK_MONOTONIC=0`**:
在 ARM GCC 工具链中必须定义此宏——`time.h` 中 `CLOCK_MONOTONIC` 通常未定义，但 micro\-ROS 代码引用了它。缺少此定义会导致编译报错。

### 连接稳定性

|问题|解决|
|---|---|
|Ping 超时频繁断开|将 ping 超时从 100ms 增至 500ms|
|Publisher 丢消息|使用 Reliable QoS 而非 Best\-Effort|
|初始连接失败|确认 UART 波特率匹配（921600），确认 RX/TX 引脚方向|
|DMA 接收丢数据|环形缓冲区大小从 512 增至 2048|

### 静态库构建

- **在 Linux 上构建**: 强烈推荐。Windows 上构建会遇到路径、工具链等诸多问题

- **工具链匹配**: 静态库编译选项（`-mcpu`, `-mfpu`, `-mfloat-abi`）必须与 MCU 项目一致

- **C 标准库**: micro\-ROS 依赖的 `clock_gettime()`、`pthread` 等 POSIX API 由 FreeRTOS\-POSIX 提供（而非用户自行实现）；`malloc`/`free` 由 FreeRTOS 堆实现（`heap_4.c`）提供，实际调用 `pvPortMalloc`/`vPortFree`

### FreeRTOS\-POSIX include 路径遗漏

**症状**: 编译通过但运行出现异常行为（如上位机agent启动后无消息），无明显编译错误或链接错误。此类问题极难排查，因为所有代码均能正常编译，错误完全是运行时隐式的。

**原因**: FreeRTOS\-POSIX 的 STM32 平台移植头文件路径 `portable/st/stm32l475_discovery` 未添加到 CMake include 路径中。该目录下的 `FreeRTOS_POSIX_portable.h` 是平台相关类型定义和宏的入口——缺失时编译器不会报错（取决于工具链警告级别），但 FreeRTOS\-POSIX 内部的部分平台相关定义可能未被正确解析，导致运行时的隐式行为差异。

**修复**: 确保 CMake 中包含以下路径（详见 §3\.3\.3）：

```cmake
target_include_directories(FreeRTOS-POSIX PUBLIC
    # ... 其他路径 ...
    "${CMAKE_CURRENT_SOURCE_DIR}/FreeRTOS-POSIX/FreeRTOS-Plus-POSIX/include/portable/st/stm32l475_discovery"
)
```

> **注意**: 即使 MCU 是 STM32H7，仍然使用 `stm32l475_discovery` 这个路径——这是 FreeRTOS\-Plus\-POSIX 中 ST 官方支持的唯一 STM32 移植，各 STM32 系列的 POSIX 移植定义完全一致（均沿用默认配置）。
> 
> 

### BSP UART 配置

- **UART 列表顺序**: BSP 中 UART 编号必须与实际硬件对应（commit `59834b4`）

- **波特率**: 921600（非标准波特率，需要 MCU 时钟支持）

- **DMA vs 中断**: 高频率通信推荐使用 DMA（本项目使用 `UARTBuffer<ID, 512>` 作为 DMA 缓冲区）

### Agent 端

- **RMW\_IMPLEMENTATION**: 启动 Agent 前确保 `export RMW_IMPLEMENTATION=rmw_microxrcedds`

- **串口权限**: 确保用户有串口读写权限（`sudo usermod -a -G dialout $USER`）

- **单 Agent 单设备**: 一个 Agent 实例只能管理一个 MCU 设备（一个串口）

### 开发迭代流程

1. 先在 PC 端的 ROS 2 环境中测试消息格式和 topic 设计

2. 使用 `ros2 topic echo` 和 `ros2 topic pub` 验证通信

3. 在 MCU 端逐步添加 Agent（从 1 个 Publisher 开始）

4. 使用 `ros2 topic hz` 检查发布频率和丢包情况

5. 监控 FreeRTOS 栈高水位确认资源充足

---

## 7\. 参考资料

|资源|链接|
|---|---|
|micro\-ROS 官方网站（新域名）|[https://micro\.vulcanexus\.org/](https://micro.vulcanexus.org/)|
|micro\-ROS 教程（新域名）|[https://micro\.vulcanexus\.org/docs/tutorials/](https://micro.vulcanexus.org/docs/tutorials/)|
|micro\_ros\_setup 仓库|[https://github\.com/micro\-ROS/micro\_ros\_setup](https://github.com/micro-ROS/micro_ros_setup)|
|Not Black Magic: Micro\-ROS 完整教程|[https://notblackmagic\.com/bitsnpieces/micro\-ros/](https://notblackmagic.com/bitsnpieces/micro-ros/)|
|Micro\-XRCE\-DDS 文档|[https://micro\-xrce\-dds\.docs\.eprosima\.com/en/latest/](https://micro-xrce-dds.docs.eprosima.com/en/latest/)|
|ROS 2 Humble 安装指南|[https://docs\.ros\.org/en/humble/Installation/Ubuntu\-Install\-Debians\.html](https://docs.ros.org/en/humble/Installation/Ubuntu-Install-Debians.html)|
|FreeRTOS \+ micro\-ROS 官方博客|[https://www\.freertos\.org/Community/Blogs/2020/micro\-ros\-on\-freertos](https://www.freertos.org/Community/Blogs/2020/micro-ros-on-freertos)|
|FineMote micro\-ROS 上位机配置（飞书）|[micro\-ROS上位机配置步骤](https://iwin-fins.feishu.cn/wiki/YUD4wP8E3iIg64kK3Qsc5Mhynrb?from=from_copylink)|
|FineMote micro\-ROS 实现原理与方法（飞书）|[3\.2 micro\-ROS支持](https://iwin-fins.feishu.cn/wiki/PklywBY5bihMz0k4m5Tcancmnve?from=from_copylink)|
|micro\-ROS Peer\-to\-Peer 模式（开发中）|[https://github\.com/eProsima/Micro\-XRCE\-DDS\-Client/tree/feature/brokerless\_p2p](https://github.com/eProsima/Micro-XRCE-DDS-Client/tree/feature/brokerless_p2p)|
|ROS 2 QoS 设计文档|[https://design\.ros2\.org/articles/qos\.html](https://design.ros2.org/articles/qos.html)|
|RMW 多实现切换指南|[https://docs\.ros\.org/en/humble/How\-To\-Guides/Working\-with\-multiple\-RMW\-implementations\.html](https://docs.ros.org/en/humble/How-To-Guides/Working-with-multiple-RMW-implementations.html)|
|STM32 \+ CubeMX \+ CLion \+ FreeRTOS 环境配置（鱼香ROS）|[https://fishros\.org\.cn/forum/topic/2423/stm32%E7%B3%BB%E5%88%97microros%E7%8E%AF%E5%A2%83%E9%85%8D%E7%BD%AE\-%E4%BD%BF%E7%94%A8cubemx\-clion\-freertos/5](https://fishros.org.cn/forum/topic/2423/stm32系列microros环境配置-使用cubemx-clion-freertos/5)|
|micro\_ros\_stm32cubemx\_utils（官方 STM32CubeMX 集成工具集）|[https://github\.com/micro\-ROS/micro\_ros\_stm32cubemx\_utils](https://github.com/micro-ROS/micro_ros_stm32cubemx_utils)|
|freertos\_apps（官方 FreeRTOS 平台示例）|[https://github\.com/micro\-ROS/freertos\_apps](https://github.com/micro-ROS/freertos_apps)|
|micro\-ROS\-demos（官方功能演示代码库）|[https://github\.com/micro\-ROS/micro\-ROS\-demos](https://github.com/micro-ROS/micro-ROS-demos)|
|rclc（ROS 2 C 客户端库：Executor、Lifecycle、Parameter）|[https://github\.com/ros2/rclc](https://github.com/ros2/rclc)|
|Lab\-Project\-FreeRTOS\-POSIX（FreeRTOS 官方 POSIX 适配层，MIT 许可）|[https://github\.com/FreeRTOS/Lab\-Project\-FreeRTOS\-POSIX](https://github.com/FreeRTOS/Lab-Project-FreeRTOS-POSIX)|
|microros\_f103rc（社区项目：STM32F103 \+ FreeRTOS \+ micro\-ROS）|[https://github\.com/weiyi\-zjh/microros\_f103rc](https://github.com/weiyi-zjh/microros_f103rc)|

---

> **文档维护**: 本文档基于 FineMote `feat/micro-ROS` 分支的开发历史编写。
> 如需补充或更正，请联系 IWIN\-FINS Lab或@张卫恒。
> 
> 



