//
// Created by wzj on 26-1-24.
//
// IMU 任务：SPI 读取 BMI088 + Mahony AHRS 姿态解算
//
// Robomaster_C: SPI1 DMA + 外部中断驱动, 数据到达后在调度器中处理
// MC_Board:     SPI1 轮询模式 (无 DMA), 在调度器中分频读取
//
// 调试日志:
//   2026-06-07: 将多比特位标志改为单字节原子状态机, 移除 __disable_irq().
//     Cortex-M4 上 uint8_t 读写是单条 LDRB/STRB 指令, 不可分割.
//     TIM7(优先级0) 抢占 DMA ISR(优先级1) 时, IMU_Task() 读到的字节
//     要么是旧状态, 要么是新状态, 不会看到中间值 → 无需关中断.
//     SPI 初始化恢复为 .bak 的行内快速配置 (已验证可用), 预分频32 (5.25MHz, 合规).
//

#include "IMUTask.h"
#include "main.h"
#include "BMI088.h"
#include "BMI088reg.h"
#include "MahonyAHRS.h"
#include "Scheduler.h"

volatile int g_step = 0;
volatile int g_gyro_cnt = 0;
volatile int g_exti_cnt = 0;
volatile int g_accel_exti_cnt = 0;  /* 加计 EXTI 单独计数 */
volatile int g_dma_err = 0;
volatile int g_dr_drop = 0;         /* DR 脉冲丢弃计数 (非 IDLE 状态到达) */
volatile int g_buf_conflict = 0;    /* 缓冲区竞态计数 */
volatile uint16_t g_spi_sr = 0;     /* SPI1->SR 快照 (诊断) */
volatile int g_spi_bsy_cnt = 0;     /* BSY 等待累计循环次数 */
volatile int g_spi_bsy_timeout = 0; /* BSY 等待超时次数 ( >0 = SPI 卡死) */
volatile uint16_t g_spi_sr_bsy = 0; /* BSY 事件时的 SPI_SR 完整快照 */
volatile float gyro_peak[3] = {0};  /* 陀螺峰值 (abs, rad/s) */
volatile float accel_peak[3] = {0}; /* 加计峰值 (abs, m/s²) */
volatile float q_norm = 1.0f;       /* 四元数模长 (应 ≈1.0, 偏离 → 发散) */
volatile int g_nan_cnt = 0;         /* NaN 检测计数 (四元数或欧拉角出现 NaN) */
volatile int g_zero_accel_cnt = 0;  /* 加计近零计数 (可能触发 invSqrt(0)→NaN) */
volatile int g_sensor_fault = 0;    /* 传感器数据越界标志 (0=正常, 1=陀螺越界, 2=加计越界) */
volatile int g_sensor_reset_cnt = 0;/* BMI088 软复位次数 */
volatile uint8_t g_raw_gyro_snap[8]; /* 异常时陀螺 DMA 缓冲区快照 */
volatile uint8_t g_raw_accel_snap[9];/* 异常时加计 DMA 缓冲区快照 */
#ifdef __ROBOMASTER_C
#include "BSP_SPI.h"
#endif

#include <cmath>
#include <cstdio>

/* ── 板级 LED 引脚适配 ────────────────────────────────────── */
#ifdef __ROBOMASTER_C
/* Robomaster_C 使用红色 LED (CubeMX main.h 中定义) */
#define BLINK_LED_PORT  LED_R_GPIO_Port
#define BLINK_LED_PIN   LED_R_Pin
#elif defined(__MC_BOARD)
/* MC_Board: 包含 MC_Board.h 获取 LED_GPIO_Port/LED_Pin 宏定义 */
#include "MC_Board.h"
#define BLINK_LED_PORT  LED_GPIO_Port
#define BLINK_LED_PIN   LED_Pin
#endif

#ifdef __cplusplus
extern "C" {
#endif
extern SPI_HandleTypeDef hspi1;
extern IWDG_HandleTypeDef hiwdg;
extern UART_HandleTypeDef huart1;
#ifdef __cplusplus
}
#endif

/* ── DMA 缓冲区 (仅 Robomaster_C) ─────────────────────────── */
#ifdef __ROBOMASTER_C

uint8_t gyro_dma_rx_buf[SPI_DMA_GYRO_LENGHT];
uint8_t gyro_dma_tx_buf[SPI_DMA_GYRO_LENGHT] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_dma_rx_buf[SPI_DMA_ACCEL_LENGHT];
uint8_t accel_dma_tx_buf[SPI_DMA_ACCEL_LENGHT] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_temp_dma_rx_buf[SPI_DMA_ACCEL_TEMP_LENGHT];
uint8_t accel_temp_dma_tx_buf[SPI_DMA_ACCEL_TEMP_LENGHT] = {0xA2, 0xFF, 0xFF, 0xFF};

/* ── 原子状态变量: 单字节读写, Cortex-M4 不可分割 ──────────── */
volatile uint8_t gyro_state = IMU_STATE_IDLE;
volatile uint8_t accel_state = IMU_STATE_IDLE;
volatile uint8_t accel_temp_state = IMU_STATE_IDLE;
volatile uint8_t imu_start_dma_flag = 0;

static void imu_cmd_spi_dma(void);

#endif /* __ROBOMASTER_C */

/* ── 共享数据 ─────────────────────────────────────────────── */
volatile bmi088_real_data_t bmi088_real_data;
volatile float INS_quat[4] = {1.0f, 0.0f, 0.0f, 0.0f};
volatile float INS_angle[3] = {0.0f, 0.0f, 0.0f};
volatile float gyro_avg[3] = {0.0f, 0.0f, 0.0f};  /* 陀螺指数移动平均 */

/* ── AHRS 工具函数 ────────────────────────────────────────── */
static void AHRS_init(volatile float quat[4], float accel[3]) {
    (void) accel;
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}

static void AHRS_update(volatile float quat[4], float time, float gyro[3], float accel[3]) {
    (void) time;
    MahonyAHRSupdateIMU((float *)quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

static void get_angle(volatile float q[4], float *yaw, float *pitch, float *roll) {
    *yaw   = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll  = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/* ── 数据访问器 ───────────────────────────────────────────── */
const volatile float *get_INS_quat_point()   { return INS_quat; }
const volatile float *get_INS_angle_point()  { return INS_angle; }
const volatile float *get_gyro_data_point()  { return bmi088_real_data.gyro; }
const volatile float *get_accel_data_point() { return bmi088_real_data.accel; }
const volatile float *get_mag_data_point()   { return nullptr; }  // 未接磁力计

/* ── SPI 诊断工具 ─────────────────────────────────────────── */
static bool BMI088_spi_read(uint8_t reg, uint8_t *val, bool is_gyro) {
    if (is_gyro) {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
    } else {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
    }
    HAL_IWDG_Refresh(&hiwdg);
    uint8_t tx, rx;
    tx = reg | 0x80;
    if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10) != HAL_OK) goto fail;
    if (is_gyro) {
        tx = 0x55;
        if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10) != HAL_OK) goto fail;
    } else {
        tx = 0x55;
        HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10); // dummy
        tx = 0x55;
        if (HAL_SPI_TransmitReceive(&hspi1, &tx, &rx, 1, 10) != HAL_OK) goto fail;
    }
    *val = rx;
    if (is_gyro) {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    return true;
fail:
    if (is_gyro) {
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
    } else {
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
    }
    return false;
}

static void safe_delay(uint32_t ms) {
    uint32_t start = HAL_GetTick();
    while (HAL_GetTick() - start < ms) {
        HAL_IWDG_Refresh(&hiwdg);
    }
}

static void blink_led(int n) {
    for (int i = 0; i < n; i++) {
        HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, GPIO_PIN_SET);
        safe_delay(200);
        HAL_GPIO_WritePin(BLINK_LED_PORT, BLINK_LED_PIN, GPIO_PIN_RESET);
        safe_delay(200);
    }
    safe_delay(1000);
}

/* ── 初始化 ───────────────────────────────────────────────── */
void INS_init() {
    uint8_t chip_id;

    /* ── CubeMX 已配 SPI1 为 Mode 3, prescaler 256 (≈656kHz).
     *     用低速 SPI 完成芯片验证和寄存器初始化,
     *     确保 BMI088 有足够时间处理每个字节 (低速下字节间 ~12μs).
     *     初始化完成后再切到高速 SPI 做 DMA. */
    g_step = 1;

    /* Step 1: 验证芯片 ID, 兼容冷/热启动 (最多重试 10 次, 每次 100ms) */
    g_step = 2;
    for (int retry = 0; retry < 10; retry++) {
        BMI088_delay_ms(retry == 0 ? 500 : 100);

        /* Accel Chip ID (reg 0x00 → 0x1E) */
        if (!BMI088_spi_read(0x00, &chip_id, false) || chip_id != 0x1E) {
            if (retry < 9) continue;
            while (1) { blink_led(1); }  /* 1 blink = Accel comm fail */
        }

        /* Gyro Chip ID (reg 0x00 → 0x0F) */
        if (!BMI088_spi_read(0x00, &chip_id, true) || chip_id != 0x0F) {
            if (retry < 9) continue;
            while (1) { blink_led(2); }  /* 2 blinks = Gyro comm fail */
        }

        break;  /* both chip IDs OK */
    }
    g_step = 3;

    /* Step 3: 用 BMI088 驱动初始化加速度计 (含回读验证, 写入失败会报错) */
    {
        uint8_t accel_ret = bmi088_accel_init();
        if (accel_ret != BMI088_NO_ERROR) {
            /* 用 blink 次数指示失败的寄存器索引 */
            while (1) { blink_led(accel_ret + 1); }
        }
    }
    g_step = 4;

    /* Step 4: 用 BMI088 驱动初始化陀螺仪 (含回读验证) */
    {
        uint8_t gyro_ret = bmi088_gyro_init();
        if (gyro_ret != BMI088_NO_ERROR) {
            while (1) { blink_led(gyro_ret + 10); }  /* 10+ 次闪烁 = 陀螺错误 */
        }
    }
    g_step = 5;

    /* Step 5: 读取初始数据 + AHRS 初始化 (使用默认低速 SPI) */
    BMI088_read((float *)bmi088_real_data.gyro, (float *)bmi088_real_data.accel, (float *)&bmi088_real_data.temp);
    AHRS_init(INS_quat, (float *)bmi088_real_data.accel);
    g_step = 6;

    /* 验证读数: 阻塞式读取一次, 存储 gyro X 到 g_gyro_cnt (单位: mrad/s) */
    { float g[3], a[3], t;
      BMI088_read(g, a, &t);
      g_gyro_cnt = (int)(g[0] * 1000);
    }
    g_step = 8;

    /* ── Step 6: 切换到高速 SPI 模式, 启动 DMA ──────────────────
     *     初始化完成后再提速: Mode 3, prescaler 32 → ~5.25MHz (≤10MHz 合规) */
    hspi1.Init.CLKPolarity = SPI_POLARITY_HIGH;
    hspi1.Init.CLKPhase = SPI_PHASE_2EDGE;
    hspi1.Init.NSS = SPI_NSS_SOFT;
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_32;
    if (HAL_SPI_Init(&hspi1) != HAL_OK) {
        Error_Handler();
    }

#ifdef __ROBOMASTER_C
    SPI1_DMA_init((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
    g_step = 9;
    imu_start_dma_flag = 1;
#endif
}


/* ── 调度器任务 (每 1ms 由 FineMoteScheduler 调用) ─────────── */
void IMU_Task() {
#ifdef __ROBOMASTER_C
    /* ── 传感器故障自动恢复 ──────────────────────────────────
     *     检测到数据越界 (SPI 反同步) 后, 暂停 DMA,
     *     软复位加速度计, 恢复 DMA. 过程约 200ms,
     *     期间 IWDG 由 BMI088_delay_ms 自动刷新. */
    if (g_sensor_fault) {
        imu_start_dma_flag = 0;        // 暂停 DMA
        g_sensor_fault = 0;
        g_sensor_reset_cnt++;
        bmi088_accel_init();           // 软复位 + 重配 (含 80ms 等待)
        BMI088_read((float *)bmi088_real_data.gyro,
                    (float *)bmi088_real_data.accel,
                    (float *)&bmi088_real_data.temp);
        /* 重置四元数到初始姿态, 用新的加计数据 */
        AHRS_init(INS_quat, (float *)bmi088_real_data.accel);
        imu_start_dma_flag = 1;        // 恢复 DMA
        return;  /* 跳过本周期 (数据已被 BMI088_read 更新) */
    }

    /* --- DMA 模式: 检查原子状态, 无需关中断 ---
     *
     * 以陀螺 (1000Hz) 为主时钟驱动 AHRS 解算.
     * 加计 (1600Hz) 和陀螺是异步传感器, 不要求配对同步.
     * AHRS 运行时使用的是 bmi088_real_data 中最新的加计数据 —
     * 即使比陀螺数据旧几百微秒, 重力方向几乎不变,
     * Mahony 滤波器的比例修正完全能容忍这种微小偏差. */
    bool new_gyro = false;

    if (gyro_state == IMU_STATE_DATA_READY) {
        gyro_state = IMU_STATE_IDLE;
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET,
                              (float *)bmi088_real_data.gyro);
        /* 应用常量偏置 (抵消稳态漂移) */
        bmi088_real_data.gyro[0] -= GYRO_BIAS_X;
        bmi088_real_data.gyro[1] -= GYRO_BIAS_Y;
        bmi088_real_data.gyro[2] -= GYRO_BIAS_Z;
        /* 指数移动平均 (CubeMonitor 观察偏置用) */
        gyro_avg[0] += GYRO_AVG_ALPHA * (bmi088_real_data.gyro[0] - gyro_avg[0]);
        gyro_avg[1] += GYRO_AVG_ALPHA * (bmi088_real_data.gyro[1] - gyro_avg[1]);
        gyro_avg[2] += GYRO_AVG_ALPHA * (bmi088_real_data.gyro[2] - gyro_avg[2]);
        /* 数据越界检测: 陀螺量程 ±2000dps ≈ ±34.9 rad/s */
        { float gx = fabsf(bmi088_real_data.gyro[0]);
          float gy = fabsf(bmi088_real_data.gyro[1]);
          float gz = fabsf(bmi088_real_data.gyro[2]);
          if (gx > gyro_peak[0]) gyro_peak[0] = gx;
          if (gy > gyro_peak[1]) gyro_peak[1] = gy;
          if (gz > gyro_peak[2]) gyro_peak[2] = gz;
          if (gx > 34.9f || gy > 34.9f || gz > 34.9f) {
              if (!g_sensor_fault) { /* 首次故障时保存现场 */
                  g_sensor_fault = 1;
                  for (int i=0;i<8;i++) g_raw_gyro_snap[i]=gyro_dma_rx_buf[i];
                  for (int i=0;i<9;i++) g_raw_accel_snap[i]=accel_dma_rx_buf[i];
              }
          }
        }
        g_gyro_cnt++;
        new_gyro = true;
    }
    if (accel_state == IMU_STATE_DATA_READY) {
        accel_state = IMU_STATE_IDLE;
        BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                               (float *)bmi088_real_data.accel, (float *)&bmi088_real_data.time);
        /* 峰值 + 越界检测: 加计量程 ±3G ≈ ±29.4 m/s² */
        { float ax = fabsf(bmi088_real_data.accel[0]);
          float ay = fabsf(bmi088_real_data.accel[1]);
          float az = fabsf(bmi088_real_data.accel[2]);
          if (ax > accel_peak[0]) accel_peak[0] = ax;
          if (ay > accel_peak[1]) accel_peak[1] = ay;
          if (az > accel_peak[2]) accel_peak[2] = az;
          if (ax > 29.4f || ay > 29.4f || az > 29.4f) {
              if (!g_sensor_fault) {
                  g_sensor_fault = 2;
                  for (int i=0;i<8;i++) g_raw_gyro_snap[i]=gyro_dma_rx_buf[i];
                  for (int i=0;i<9;i++) g_raw_accel_snap[i]=accel_dma_rx_buf[i];
              }
          }
        }
    }
    if (accel_temp_state == IMU_STATE_DATA_READY) {
        accel_temp_state = IMU_STATE_IDLE;
        BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET,
                                     (float *)&bmi088_real_data.temp);
    }

    if (!new_gyro) return; // 无新陀螺数据则跳过 AHRS 更新
#elif defined(__MC_BOARD)
    /* --- 轮询模式: 每 10ms 读一次 (100Hz), 留给其他任务时间 --- */
    static uint32_t poll_cnt = 0;
    if (++poll_cnt < 10) {
        return;
    }
    poll_cnt = 0;
    BMI088_read((float *)bmi088_real_data.gyro, (float *)bmi088_real_data.accel, (float *)&bmi088_real_data.temp);
#endif

    // 运行 AHRS 更新 + 欧拉角解算
    {
        float ax = bmi088_real_data.accel[0];
        float ay = bmi088_real_data.accel[1];
        float az = bmi088_real_data.accel[2];
        float accel_mag2 = ax*ax + ay*ay + az*az;
        /* 加计近零保护: 如果加计向量模长接近 0, invSqrt→inf→NaN,
         *     跳过本次 AHRS 更新, 只用陀螺积分维持姿态 (一个周期影响可忽略). */
        if (accel_mag2 < 0.01f) {  /* < 0.1g → 异常 */
            g_zero_accel_cnt++;
        } else {
            AHRS_update(INS_quat, 0.001f, (float *)bmi088_real_data.gyro, (float *)bmi088_real_data.accel);
        }
    }
    /* NaN 检测: 四元数出现 NaN 则重置滤波器 */
    if (INS_quat[0] != INS_quat[0]) {  /* NaN != NaN 恒为真 */
        g_nan_cnt++;
        INS_quat[0] = 1.0f; INS_quat[1] = 0.0f;
        INS_quat[2] = 0.0f; INS_quat[3] = 0.0f;
    }
    /* 四元数模长 */
    q_norm = sqrtf(INS_quat[0]*INS_quat[0] + INS_quat[1]*INS_quat[1]
                 + INS_quat[2]*INS_quat[2] + INS_quat[3]*INS_quat[3]);
    get_angle(INS_quat,
              (float *)(INS_angle + INS_YAW_ADDRESS_OFFSET),
              (float *)(INS_angle + INS_PITCH_ADDRESS_OFFSET),
              (float *)(INS_angle + INS_ROLL_ADDRESS_OFFSET));

    /* ── 调试串口输出 ────────────────────────────────────────
     * 启用: IMU_DEBUG_UART=1 (在 IMUTask.h 中配置)
     * 输出: g_step(系统tick), g_gyro_cnt, g_exti_cnt, g_dma_err, yaw, pitch, roll
     *       g_step 在 init 阶段为 0-9, 完成后由 MainRTLoop 每 1ms 递增 (系统运行计数)
     * 频率: 每 100ms (每 100 次调度器调用)
     * 波特率: 115200, 每次输出 ~1.5ms 阻塞, 对 1ms 调度器有轻微影响 */
#if IMU_DEBUG_UART
    {
        static uint32_t uart_cnt = 0;
        if (++uart_cnt >= 100) {
            uart_cnt = 0;
            float yaw_deg   = INS_angle[0] * 57.29578f;
            float pitch_deg = INS_angle[1] * 57.29578f;
            float roll_deg  = INS_angle[2] * 57.29578f;
            char buf[96];
            int len = snprintf(buf, sizeof(buf),
                "T:%d G:%d AE:%d D:%d DRdrop:%d | YPR:%.2f,%.2f,%.2f\r\n",
                g_step, g_gyro_cnt, g_accel_exti_cnt, g_dma_err, g_dr_drop,
                (double)yaw_deg, (double)pitch_deg, (double)roll_deg);
            if (len > 0 && len < (int)sizeof(buf)) {
                HAL_UART_Transmit(&huart1, (uint8_t *)buf, len, 10);
            }
        }
    }
#endif
}
TASK_EXPORT(IMU_Task);

/* ── 校准接口 (暂未实现完整校准流程) ───────────────────────── */
void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count) {
    (void) cali_scale;
    (void) cali_offset;
    (void) time_count;
}

void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3]) {
    (void) cali_scale;
    (void) cali_offset;
}

/* ═══════════════════════════════════════════════════════════
 * 以下为中断服务例程 — 仅 Robomaster_C 使用 DMA 路径
 * MC_Board 走轮询路径, 不涉及以下代码
 *
 * 原子性设计:
 *   每个传感器通道使用单字节 uint8_t 状态变量.
 *   Cortex-M4 上 LDRB/STRB 是单条不可分割指令.
 *   TIM7(优先级0, 最高) 可抢占 DMA ISR(优先级1), 但:
 *   - DMA ISR 写入状态 (STRB) → IMU_Task 读到旧值或新值, 不会是中间值
 *   - IMU_Task 清除状态 (STRB) → DMA ISR 读到旧值或新值, 两者皆合法
 *   因此全程无需 __disable_irq().
 * ═══════════════════════════════════════════════════════════ */

#ifdef __ROBOMASTER_C

/* ── GPIO 外部中断回调 (BMI088 数据就绪引脚) ──────────────── */
#ifdef __cplusplus
extern "C" {
#endif
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {
    g_exti_cnt++;
    /* IDLE 检查: 仅在空闲状态接受新 DR, 防止 DMA 缓冲区在 IMU_Task
     * 读取期间被新传输覆盖. 如果 DR 到达时状态非 IDLE, 丢弃本次 DR
     * (传感器在下一采样周期自动补发). 丢弃的 DR 计入 g_dr_drop. */
    if (GPIO_Pin == INT1_ACCEL_Pin) {
        g_accel_exti_cnt++;
        if (accel_state == IMU_STATE_IDLE)
            accel_state = IMU_STATE_DR_PENDING;
        else
            g_dr_drop++;
        if (accel_temp_state == IMU_STATE_IDLE)
            accel_temp_state = IMU_STATE_DR_PENDING;
        else
            g_dr_drop++;
    } else if (GPIO_Pin == INT1_GYRO_Pin) {
        if (gyro_state == IMU_STATE_IDLE)
            gyro_state = IMU_STATE_DR_PENDING;
        else
            g_dr_drop++;
    }
    if (imu_start_dma_flag) {
        imu_cmd_spi_dma();
    }
}

/* ── SPI DMA 状态机 (仲裁 gyro / accel / temp 三路传输) ────── */
static void imu_cmd_spi_dma() {
    /* 快速预检: DMA 硬件是否空闲? (需要总线空闲才能启动新传输) */
    if ((hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) ||
        (hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)) {
        return;
    }

    /* 优先级: 陀螺 > 加速度计 > 温度
     * 每个状态转换是单条 STRB → 原子操作 */
    if (gyro_state == IMU_STATE_DR_PENDING) {
        gyro_state = IMU_STATE_DMA_ACTIVE;
        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) gyro_dma_tx_buf, (uint32_t) gyro_dma_rx_buf, SPI_DMA_GYRO_LENGHT);
        return;
    }

    if (accel_state == IMU_STATE_DR_PENDING) {
        accel_state = IMU_STATE_DMA_ACTIVE;
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) accel_dma_tx_buf, (uint32_t) accel_dma_rx_buf, SPI_DMA_ACCEL_LENGHT);
        return;
    }

    if (accel_temp_state == IMU_STATE_DR_PENDING) {
        accel_temp_state = IMU_STATE_DMA_ACTIVE;
        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t) accel_temp_dma_tx_buf, (uint32_t) accel_temp_dma_rx_buf, SPI_DMA_ACCEL_TEMP_LENGHT);
        return;
    }
}

/* ── DMA 传输完成中断 (SPI1 RX) ────────────────────────────── */
void DMA2_Stream0_IRQHandler() {
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        /* 单次 DMA 传输只能完成一个通道 → 用 else-if 互斥
         * 每次状态转换 = 单条 STRB → 原子操作, 不关中断.
         *
         * ★ 关键: 拉高 CS 前必须等 SPI BSY=0.
         * RX DMA 计数器归零触发 TC 中断时, TX DMA 可能还在发最后一字节,
         * SPI 外设 BSY 标志可能仍为 1. 此时拉高 CS 会截断 BMI088 的事务,
         * 导致传感器进入异常状态并持续输出脏数据 (表现为震荡). */
        if (gyro_state == IMU_STATE_DMA_ACTIVE) {
            gyro_state = IMU_STATE_DATA_READY;
            { int to = 10000; while ((hspi1.Instance->SR & SPI_SR_BSY) && --to) g_spi_bsy_cnt++; if (!to) { g_spi_bsy_timeout++; g_spi_sr_bsy = hspi1.Instance->SR; } }
            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);
        } else if (accel_state == IMU_STATE_DMA_ACTIVE) {
            accel_state = IMU_STATE_DATA_READY;
            { int to = 10000; while ((hspi1.Instance->SR & SPI_SR_BSY) && --to) g_spi_bsy_cnt++; if (!to) { g_spi_bsy_timeout++; g_spi_sr_bsy = hspi1.Instance->SR; } }
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        } else if (accel_temp_state == IMU_STATE_DMA_ACTIVE) {
            accel_temp_state = IMU_STATE_DATA_READY;
            { int to = 10000; while ((hspi1.Instance->SR & SPI_SR_BSY) && --to) g_spi_bsy_cnt++; if (!to) { g_spi_bsy_timeout++; g_spi_sr_bsy = hspi1.Instance->SR; } }
            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        /* ★ CS 最小高电平时间: 加计和温度共用 CS_ACCEL.
         *     加计 DMA 完成后 CS 拉高, 温度 DMA 即将拉低同一个 CS.
         *     BMI088 要求两次事务之间 CS 保持高电平 ≥ 1-2μs,
         *     此处延迟确保 CS 高电平时间满足要求. */
        { volatile int cs_dly = 500; while (--cs_dly) {} }
        /* 链式启动下一个挂起的 DMA 传输 */
        imu_cmd_spi_dma();
    }

    /* 记录 DMA 传输/ FIFO 错误, 同时捕获 SPI 状态寄存器 */
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TE_FLAG_INDEX(hspi1.hdmarx));
        g_spi_sr = hspi1.Instance->SR;
        g_dma_err++;
    }
    if (__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx)) != RESET) {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_FE_FLAG_INDEX(hspi1.hdmarx));
        g_spi_sr = hspi1.Instance->SR;
        g_dma_err++;
    }
}
#ifdef __cplusplus
}
#endif

#endif /* __ROBOMASTER_C */
