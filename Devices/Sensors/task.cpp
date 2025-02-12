#include "main.h"
#include "task.h"
#include "bsp_spi.h"
#include "MahonyAHRS.h"
#include "BMI088.h"
// #include "ist8310driver.h"
#include "cmath"
#include "pid.h"

void imu_pwm_set(uint16_t pwm)
{
    TIM10->CCR1 = (pwm);
}
#define IMU_temp_PWM(pwm)  imu_pwm_set(pwm)                    //pwm����

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
// volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

uint8_t gyro_dma_rx_buf[8];
uint8_t gyro_dma_tx_buf[8] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_dma_rx_buf[9];
uint8_t accel_dma_tx_buf[9] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_temp_dma_rx_buf[4];
uint8_t accel_temp_dma_tx_buf[4] = {0xA2, 0xFF, 0xFF, 0xFF};

static uint8_t first_temperate;
static const float imu_temp_PID[3] = {1600.0f, 0.2f, 0.0f};

float INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
float INS_angle[3] = {0.0f, 0.0f, 0.0f};      //ŷ���� ��λ rad

extern SPI_HandleTypeDef hspi2;
bmi088_real_data_t bmi088_real_data;
// ist8310_real_data_t ist8310_real_data;
// static pid_type_def imu_temp_pid;

static void imu_temp_control(float temp);
static void imu_cmd_spi_dma();
static void AHRS_init(float quat[4], float accel[3], float mag[3]);
static void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3]);
static void get_angle(float q[4], float* yaw, float* pitch, float* roll);

SPI_WITH_DMA_t spi_dma = {
    .spiHandle = &hspi2,
    .rxDMAHandle = &hdma_spi2_rx,
    .txDMAHandle = &hdma_spi2_tx
};

void INS_task_Init()
{
    while(BMI088_init()) {};
    // while(ist8310_init()) {};
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    // PID_init(&imu_temp_pid, PID_POSITION, imu_temp_PID, 4500.0f, 4400.0f);
    // AHRS_init(INS_quat, bmi088_real_data.accel, ist8310_real_data.mag);
    // hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    // SPI1_DMA_init((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);
    SPI_DMA_init(spi_dma,(uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);
    imu_start_dma_flag = 1;
}

void INS_task()
{
    if(gyro_update_flag & (1 << IMU_NOTIFY_SHFITS))
    {
        gyro_update_flag &= ~(1 << IMU_NOTIFY_SHFITS);
        BMI088_gyro_read_over(gyro_dma_rx_buf + BMI088_GYRO_RX_BUF_DATA_OFFSET, bmi088_real_data.gyro);
    }

    if(accel_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_accel_read_over(accel_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, bmi088_real_data.accel, &bmi088_real_data.time);
    }

    if(accel_temp_update_flag & (1 << IMU_UPDATE_SHFITS))
    {
        accel_temp_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
        BMI088_temperature_read_over(accel_temp_dma_rx_buf + BMI088_ACCEL_RX_BUF_DATA_OFFSET, &bmi088_real_data.temp);
        imu_temp_control(bmi088_real_data.temp);
    }

    AHRS_update(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel, ist8310_real_data.mag);
    get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
}

static void AHRS_init(float quat[4], float accel[3], float mag[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}
static void AHRS_update(float quat[4], float time, float gyro[3], float accel[3], float mag[3])
{
    MahonyAHRSupdate(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2], mag[0], mag[1], mag[2]);
}
static void get_angle(float q[4], float* yaw, float* pitch, float* roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}

/**
  * @brief          ����bmi088���¶�
  * @param[in]      temp:bmi088���¶�
  */
static void imu_temp_control(float temp)
{
    uint16_t tempPWM;
    static uint8_t temp_constant_time = 0;
    if(first_temperate)
    {
        PID_calc(&imu_temp_pid, temp, 45.0f);
        if(imu_temp_pid.out < 0.0f)
        {
            imu_temp_pid.out = 0.0f;
        }
        tempPWM = (uint16_t)imu_temp_pid.out;
        IMU_temp_PWM(tempPWM);
    }
    else
    {
        //��û�дﵽ���õ��¶ȣ�һֱ����ʼ���
        if(temp > 45.0f)
        {
            temp_constant_time++;
            if(temp_constant_time > 200)
            {
                //�ﵽ�����¶ȣ�������������Ϊһ������ʣ���������
                first_temperate = 1;
                imu_temp_pid.Iout = 5000 / 2.0f;
            }
        }
        IMU_temp_PWM(5000 - 1);
    }
}

static void imu_cmd_spi_dma(void)
{
    //���������ǵ�DMA����
    if((gyro_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(accel_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        gyro_update_flag &= ~(1 << IMU_DR_SHFITS);
        gyro_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)gyro_dma_tx_buf, (uint32_t)gyro_dma_rx_buf, 8);
        return;
    }
    //�������ٶȼƵ�DMA����
    if((accel_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_temp_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_dma_tx_buf, (uint32_t)accel_dma_rx_buf, 9);
        return;
    }
    if((accel_temp_update_flag & (1 << IMU_DR_SHFITS)) && !(hspi1.hdmatx->Instance->CR & DMA_SxCR_EN) && !(hspi1.hdmarx->Instance->CR & DMA_SxCR_EN)
            && !(gyro_update_flag & (1 << IMU_SPI_SHFITS)) && !(accel_update_flag & (1 << IMU_SPI_SHFITS)))
    {
        accel_temp_update_flag &= ~(1 << IMU_DR_SHFITS);
        accel_temp_update_flag |= (1 << IMU_SPI_SHFITS);

        HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_RESET);
        SPI1_DMA_enable((uint32_t)accel_temp_dma_tx_buf, (uint32_t)accel_temp_dma_rx_buf, 4);
        return;
    }
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == INT1_ACCEL_Pin)
    {
        accel_update_flag |= 1 << IMU_DR_SHFITS;
        accel_temp_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == INT1_GRYO_Pin)
    {
        gyro_update_flag |= 1 << IMU_DR_SHFITS;
        if(imu_start_dma_flag)
        {
            imu_cmd_spi_dma();
        }
    }
    else if(GPIO_Pin == IST8310_DRDY_Pin)
    {
        mag_update_flag |= 1 << IMU_DR_SHFITS;

        if(mag_update_flag &= 1 << IMU_DR_SHFITS)
        {
            mag_update_flag &= ~(1 << IMU_DR_SHFITS);
            mag_update_flag |= (1 << IMU_SPI_SHFITS);

            ist8310_read_mag(ist8310_real_data.mag);
        }
    }
}

void DMA2_Stream2_IRQHandler(void)
{
    if(__HAL_DMA_GET_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx)) != RESET)
    {
        __HAL_DMA_CLEAR_FLAG(hspi1.hdmarx, __HAL_DMA_GET_TC_FLAG_INDEX(hspi1.hdmarx));

        //gyro read over
        //�����Ƕ�ȡ���
        if(gyro_update_flag & (1 << IMU_SPI_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_SPI_SHFITS);
            gyro_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_GYRO_GPIO_Port, CS1_GYRO_Pin, GPIO_PIN_SET);

        }
        //���ٶȼƶ�ȡ���
        if(accel_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }
        //�¶ȶ�ȡ���
        if(accel_temp_update_flag & (1 << IMU_SPI_SHFITS))
        {
            accel_temp_update_flag &= ~(1 << IMU_SPI_SHFITS);
            accel_temp_update_flag |= (1 << IMU_UPDATE_SHFITS);

            HAL_GPIO_WritePin(CS1_ACCEL_GPIO_Port, CS1_ACCEL_Pin, GPIO_PIN_SET);
        }

        imu_cmd_spi_dma();

        if(gyro_update_flag & (1 << IMU_UPDATE_SHFITS))
        {
            gyro_update_flag &= ~(1 << IMU_UPDATE_SHFITS);
            gyro_update_flag |= (1 << IMU_NOTIFY_SHFITS);
            __HAL_GPIO_EXTI_GENERATE_SWIT(GPIO_PIN_0);
        }
    }
}

