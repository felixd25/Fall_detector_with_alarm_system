//Generated on 2024/11/16
// MPU6050 Data collected from MPU-6000 and MPU-6050 Product Specification Revision 3.4 [1]
//[1] InvenSense, “MPU-6000 and MPU-6050 Product Specification Revision 3.4,” PS-MPU-6000A-00, 08/19/2013.

#ifndef APP_MPU6050_H
#define APP_MPU6050_H

#define PI 3.14159265358979323846
#define DT 0.005    //Unit time
#define ALPHA 0.95238

#define MPU6050     0xd0    // MPU6050 ID (01101000) when AD0 set to 0 (Low electric level)

#define READ_TIME   5       //The sampling interval is 5ms, corresponding to the 200Hz sampling rate below

#define POWER_REG  0x6B     // Power management register
#define RESET_BIT   0x80    // Reset Command
#define WEAK_BIT    0x00    //Wake Command

#define SAMPLE_RATE_DIVIDER_REG 0x19    // Sample rate register
#define HZ200        0x04               // 200Hz sample rate

#define ACCEL_CONFIG_REG 0x1C       // Accelerator configuration register
#define ACCEL_FS_SEL_2G  0x00       // +-2G accelerator range
#define ACCEL_FS_SEL_4G  0x08       // +-4G accelerator range
#define ACCEL_FS_SEL_8G  0x10       // +-8G accelerator range
#define ACCEL_FS_SEL_16G 0x18       // +-16G accelerator range

#define GYRO_CONFIG_REG 0x1B        // Gyroscope configuration register
#define GYRO_FS_SEL_250DPS  0x00    // +-250DPS gyroscope range
#define GYRO_FS_SEL_500DPS  0x08    // +-500DPS gyroscope range
#define GYRO_FS_SEL_1000DPS 0x10    // +-1000DPS gyroscope range
#define GYRO_FS_SEL_2000DPS 0x18    // +-2000DPS gyroscope range

#define ACCEL_XOUT_H_REG 0x3B   // Acceleration X axis upper 8 bits
#define ACCEL_XOUT_L_REG 0x3C   // Acceleration X axis lower 8 bits
#define ACCEL_YOUT_H_REG 0x3D   // Acceleration Y axis upper 8 bits
#define ACCEL_YOUT_L_REG 0x3E   // Acceleration Y axis lower 8 bits
#define ACCEL_ZOUT_H_REG 0x3F   // Acceleration Z axis upper 8 bits
#define ACCEL_ZOUT_L_REG 0x40   // Acceleration Z axis lower 8 bits
#define TEMP_OUT_H_REG 0x41     // Temperature upper 8 bits
#define TEMP_OUT_L_REG 0x42     // Temperature lower 8 bits
#define GYRO_XOUT_H_REG 0x43    // Gyroscope X axis upper 8 bits
#define GYRO_XOUT_L_REG 0x44    // Gyroscope X axis lower 8 bits
#define GYRO_YOUT_H_REG 0x45    // Gyroscope Y axis upper 8 bits
#define GYRO_YOUT_L_REG 0x46    // Gyroscope Y axis lower 8 bits
#define GYRO_ZOUT_H_REG 0x47    // Gyroscope Z axis upper 8 bits
#define GYRO_ZOUT_L_REG 0x48    // Gyroscope Z axis lower 8 bits

# define SIG_PATH_RESET_REG 0x68    // Signal path reset register







#include "stm32f4xx.h"

void MPU6050_Init(void);

void MPU6050_Proc(void);

void MPU6050_GetResult(float *pAccel, float *pTemp, float *pGyro, float *pAngel, uint8_t *pBuffer);














#endif //APP_MPU6050_H
