#include "main.h"
#include "INS_task.h"
#include "spi.h"
#include "MahonyAHRS.h"
#include "bmi088driver.h"
#include <math.h>
#include "pid.h"

volatile uint8_t gyro_update_flag = 0;
volatile uint8_t accel_update_flag = 0;
volatile uint8_t accel_temp_update_flag = 0;
volatile uint8_t mag_update_flag = 0;
volatile uint8_t imu_start_dma_flag = 0;

uint8_t gyro_dma_rx_buf[8];
uint8_t gyro_dma_tx_buf[8] = {0x82, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_dma_rx_buf[9];
uint8_t accel_dma_tx_buf[9] = {0x92, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
uint8_t accel_temp_dma_rx_buf[4];
uint8_t accel_temp_dma_tx_buf[4] = {0xA2, 0xFF, 0xFF, 0xFF};

fp32 INS_quat[4] = {0.0f, 0.0f, 0.0f, 0.0f};
fp32 INS_angle[3] = {0.0f, 0.0f, 0.0f};      //欧拉角 单位 rad

bmi088_real_data_t bmi088_real_data;


static void AHRS_init(fp32 quat[4], fp32 accel[3]);
static void get_angle(fp32 q[4], fp32* yaw, fp32* pitch, fp32* roll);
static void AHRS_update1(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3]);

void INS_task_Init(void)
{
    while(BMI088_init()) {};
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    AHRS_init(INS_quat, bmi088_real_data.accel);
    hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_8;
    imu_start_dma_flag = 1;
}

void INS_task(void)
{
    BMI088_read(bmi088_real_data.gyro, bmi088_real_data.accel, &bmi088_real_data.temp);
    AHRS_update1(INS_quat, 0.001f, bmi088_real_data.gyro, bmi088_real_data.accel);
    get_angle(INS_quat, INS_angle + INS_YAW_ADDRESS_OFFSET, INS_angle + INS_PITCH_ADDRESS_OFFSET, INS_angle + INS_ROLL_ADDRESS_OFFSET);
    
    INS_angle[0]*=57;
    INS_angle[1]*=57;
    INS_angle[2]*=57;
}

static void AHRS_init(fp32 quat[4], fp32 accel[3])
{
    quat[0] = 1.0f;
    quat[1] = 0.0f;
    quat[2] = 0.0f;
    quat[3] = 0.0f;
}


static void AHRS_update1(fp32 quat[4], fp32 time, fp32 gyro[3], fp32 accel[3])
{
    MahonyAHRSupdateIMU(quat, gyro[0], gyro[1], gyro[2], accel[0], accel[1], accel[2]);
}

static void get_angle(fp32 q[4], fp32* yaw, fp32* pitch, fp32* roll)
{
    *yaw = atan2f(2.0f * (q[0] * q[3] + q[1] * q[2]), 2.0f * (q[0] * q[0] + q[1] * q[1]) - 1.0f);
    *pitch = asinf(-2.0f * (q[1] * q[3] - q[0] * q[2]));
    *roll = atan2f(2.0f * (q[0] * q[1] + q[2] * q[3]), 2.0f * (q[0] * q[0] + q[3] * q[3]) - 1.0f);
}



