//
// Created by wzj on 26-1-24.
//

#ifndef FINEMOTE_IMUTASK_H
#define FINEMOTE_IMUTASK_H

#include "stdint.h"


#define SPI_DMA_GYRO_LENGHT       8
#define SPI_DMA_ACCEL_LENGHT      9
#define SPI_DMA_ACCEL_TEMP_LENGHT 4


/* ── 原子状态机: 每通道一个 uint8_t, 单字节写 = Cortex-M4 原子操作 ──
 * TIM7 (优先级0) 可抢占 DMA ISR (优先级1), 但 LDRB/STRB 不可分割
 * → IMU_Task() 读到的要么是旧状态, 要么是新状态, 不会看到中间值
 */
#define IMU_STATE_IDLE          0   /* 空闲 */
#define IMU_STATE_DR_PENDING    1   /* EXTI 已触发, 等待 DMA */
#define IMU_STATE_DMA_ACTIVE    2   /* DMA 传输中 */
#define IMU_STATE_DATA_READY    3   /* DMA 完成, 数据就绪 */

extern volatile uint8_t gyro_state;
extern volatile uint8_t accel_state;
extern volatile uint8_t accel_temp_state;


#define BMI088_GYRO_RX_BUF_DATA_OFFSET  1
#define BMI088_ACCEL_RX_BUF_DATA_OFFSET 2

//ist83100原始数据在缓冲区buf的位置
#define IST8310_RX_BUF_DATA_OFFSET 16


#define TEMPERATURE_PID_KP 1600.0f //温度控制PID的kp
#define TEMPERATURE_PID_KI 0.2f    //温度控制PID的ki
#define TEMPERATURE_PID_KD 0.0f    //温度控制PID的kd

#define TEMPERATURE_PID_MAX_OUT   4500.0f //温度控制PID的max_out
#define TEMPERATURE_PID_MAX_IOUT 4400.0f  //温度控制PID的max_iout

#define MPU6500_TEMP_PWM_MAX 5000 //mpu6500控制温度的设置TIM的重载值，即给PWM最大为 MPU6500_TEMP_PWM_MAX - 1


/* ── 调试日志 ──────────────────────────────────────────────── */
extern volatile int g_step;       /* 初始化进度步骤 (0-9) */
extern volatile int g_gyro_cnt;   /* 陀螺数据消费计数 */
extern volatile int g_exti_cnt;        /* EXTI 回调触发总数 */
extern volatile int g_accel_exti_cnt;   /* 加计 EXTI 单独计数 */
extern volatile int g_dma_err;       /* DMA 错误累计 */
extern volatile int g_dr_drop;       /* DR 脉冲丢弃 (非 IDLE 到达) */
extern volatile int g_buf_conflict;  /* 缓冲区竞态计数 */
extern volatile int g_spi_bsy_cnt;    /* BSY 等待累计循环次数 */
extern volatile int g_spi_bsy_timeout;/* BSY 等待超时次数 (>0 = SPI 卡死) */
extern volatile uint16_t g_spi_sr_bsy; /* BSY 超时时的 SPI_SR 快照 */
extern volatile float gyro_peak[3];  /* 陀螺峰值 (abs, rad/s) */
extern volatile float accel_peak[3]; /* 加计峰值 (abs, m/s²) */
extern volatile float q_norm;        /* 四元数模长 (应 ≈1.0) */
extern volatile int g_nan_cnt;       /* NaN 检测计数 */
extern volatile int g_zero_accel_cnt;/* 加计近零计数 */
extern volatile int g_sensor_fault;  /* 0=正常, 1=陀螺越界, 2=加计越界 */
extern volatile int g_sensor_reset_cnt; /* BMI088 软复位次数 */
extern volatile uint8_t g_raw_gyro_snap[8];  /* 异常时陀螺 RX 快照 */
extern volatile uint8_t g_raw_accel_snap[9]; /* 异常时加计 RX 快照 */

#define IMU_DEBUG_UART  0   /* 1 = 使能 USART1 CSV 调试输出 (115200) */

/* ── 陀螺仪偏置 (rad/s) ───────────────────────────────────
 * 用于抵消稳态漂移. 观察静止时 IMU 输出的角速度, 取反填入.
 * 例: 静止时 gyro_z 读数约 0.003 rad/s → GYRO_BIAS_Z = -0.003f */
#define GYRO_BIAS_X   0.0023f
#define GYRO_BIAS_Y   (-0.0011f)
#define GYRO_BIAS_Z   0.0013f

/* ── 陀螺仪指数移动平均 (观察偏置用) ──────────────────────
 * 平滑后的角速度存在 gyro_avg[3], CubeMonitor 可直接读取.
 * GYRO_AVG_ALPHA 越小越平滑, 范围 0.0~1.0. 0.001 = 约3秒收敛. */
#define GYRO_AVG_ALPHA  0.001f
extern volatile float gyro_avg[3];   /* 平滑后角速度 (rad/s) */

#define INS_TASK_INIT_TIME 7 //任务开始初期 delay 一段时间

#define INS_YAW_ADDRESS_OFFSET    0
#define INS_PITCH_ADDRESS_OFFSET  1
#define INS_ROLL_ADDRESS_OFFSET   2

#define INS_GYRO_X_ADDRESS_OFFSET 0
#define INS_GYRO_Y_ADDRESS_OFFSET 1
#define INS_GYRO_Z_ADDRESS_OFFSET 2

#define INS_ACCEL_X_ADDRESS_OFFSET 0
#define INS_ACCEL_Y_ADDRESS_OFFSET 1
#define INS_ACCEL_Z_ADDRESS_OFFSET 2

#define INS_MAG_X_ADDRESS_OFFSET 0
#define INS_MAG_Y_ADDRESS_OFFSET 1
#define INS_MAG_Z_ADDRESS_OFFSET 2

#ifdef __cplusplus
extern "C" {
#endif
extern void INS_init();
#ifdef __cplusplus
}
#endif

/**
  * @brief          IMU 调度器任务, 由 FineMoteScheduler 每 1ms 调用
  *                 Robomaster_C: 检查 DMA 完成标志, 处理传感器数据, 执行 AHRS
  *                 MC_Board:     分频轮询 BMI088_read(), 执行 AHRS
  * @retval         none
  */
extern void IMU_Task();

/**
  * @brief          calculate gyro zero drift
  * @param[out]     cali_scale:scale, default 1.0
  * @param[out]     cali_offset:zero drift, collect the gyro ouput when in still
  * @param[out]     time_count: time, when call gyro_offset_calc 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪
  * @param[out]     陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[out]     陀螺仪的零漂，采集陀螺仪的静止的输出作为offset
  * @param[out]     陀螺仪的时刻，每次在gyro_offset调用会加1,
  * @retval         none
  */
extern void INS_cali_gyro(float cali_scale[3], float cali_offset[3], uint16_t *time_count);

/**
  * @brief          get gyro zero drift from flash
  * @param[in]      cali_scale:scale, default 1.0
  * @param[in]      cali_offset:zero drift, 
  * @retval         none
  */
/**
  * @brief          校准陀螺仪设置，将从flash或者其他地方传入校准值
  * @param[in]      陀螺仪的比例因子，1.0f为默认值，不修改
  * @param[in]      陀螺仪的零漂
  * @retval         none
  */
extern void INS_set_cali_gyro(float cali_scale[3], float cali_offset[3]);

/**
  * @brief          get the quat
  * @param[in]      none
  * @retval         the point of INS_quat
  */
/**
  * @brief          获取四元数
  * @param[in]      none
  * @retval         INS_quat的指针
  */
extern const volatile float *get_INS_quat_point();


/**
  * @brief          get the euler angle, 0:yaw, 1:pitch, 2:roll unit rad
  * @param[in]      none
  * @retval         the point of INS_angle
  */
/**
  * @brief          获取欧拉角, 0:yaw, 1:pitch, 2:roll 单位 rad
  * @param[in]      none
  * @retval         INS_angle的指针
  */
extern const volatile float *get_INS_angle_point();


/**
  * @brief          get the rotation speed, 0:x-axis, 1:y-axis, 2:roll-axis,unit rad/s
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取角速度,0:x轴, 1:y轴, 2:roll轴 单位 rad/s
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const volatile float *get_gyro_data_point();


/**
  * @brief          get aceel, 0:x-axis, 1:y-axis, 2:roll-axis unit m/s2
  * @param[in]      none
  * @retval         the point of INS_gyro
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 m/s2
  * @param[in]      none
  * @retval         INS_gyro的指针
  */
extern const volatile float *get_accel_data_point();

/**
  * @brief          get mag, 0:x-axis, 1:y-axis, 2:roll-axis unit ut
  * @param[in]      none
  * @retval         the point of INS_mag
  */
/**
  * @brief          获取加速度,0:x轴, 1:y轴, 2:roll轴 单位 ut
  * @param[in]      none
  * @retval         INS_mag的指针
  */
extern const volatile float *get_mag_data_point();

#endif //FINEMOTE_IMUTASK_H
