//
// Created by Administrator on 2024/10/3.
//

#ifndef TTEST_APP_MPU6050_H
#define TTEST_APP_MPU6050_H

#define PI 3.14159265358979323846
#define DT 0.005    //单位时间
#define ALPHA 0.95238

#define MPU6050     0xD0    // MPU6050器件ID值(01101000)   将AD0 引脚置为0

#define READ_TIME   5       //采样间隔5ms，对应下面的200Hz采样率

#define POWER_REG  0x6B     // 电源管理寄存器
#define RESET_BIT   0x80    // 复位指令
#define WEAK_BIT    0x00    //唤醒指令

#define SAMPLE_RATE_DIVIDER_REG 0x19    // 采样率寄存器
#define HZ200        0x04               // 200Hz采样率

#define ACCEL_CONFIG_REG 0x1C       // 加速度配置寄存器
#define ACCEL_FS_SEL_2G  0x00       // +-2G加速度量程
#define ACCEL_FS_SEL_4G  0x08       // +-4G加速度量程
#define ACCEL_FS_SEL_8G  0x10       // +-8G加速度量程
#define ACCEL_FS_SEL_16G 0x18       // +-16G加速度量程

#define GYRO_CONFIG_REG 0x1B        // 陀螺仪配置寄存器
#define GYRO_FS_SEL_250DPS  0x00    // +-250DPS陀螺仪量程
#define GYRO_FS_SEL_500DPS  0x08    // +-500DPS陀螺仪量程
#define GYRO_FS_SEL_1000DPS 0x10    // +-1000DPS陀螺仪量程
#define GYRO_FS_SEL_2000DPS 0x18    // +-2000DPS陀螺仪量程

#define ACCEL_XOUT_H_REG 0x3B   // 加速度X轴高8位
#define ACCEL_XOUT_L_REG 0x3C   // 加速度X轴低8位
#define ACCEL_YOUT_H_REG 0x3D   // 加速度Y轴高8位
#define ACCEL_YOUT_L_REG 0x3E   // 加速度Y轴低8位
#define ACCEL_ZOUT_H_REG 0x3F   // 加速度Z轴高8位
#define ACCEL_ZOUT_L_REG 0x40   // 加速度Z轴低8位
#define TEMP_OUT_H_REG 0x41     // 温度X轴高8位
#define TEMP_OUT_L_REG 0x42     // 温度X轴低8位
#define GYRO_XOUT_H_REG 0x43    // 陀螺仪X轴高8位
#define GYRO_XOUT_L_REG 0x44    // 陀螺仪X轴低8位
#define GYRO_YOUT_H_REG 0x45    // 陀螺仪Y轴高8位
#define GYRO_YOUT_L_REG 0x46    // 陀螺仪Y轴低8位
#define GYRO_ZOUT_H_REG 0x47    // 陀螺仪Z轴高8位
#define GYRO_ZOUT_L_REG 0x48    // 陀螺仪Z轴低8位

# define SIG_PATH_RESET_REG 0x68    // 信号路径复位寄存器






#include "stm32f1xx.h"

void MPU6050_Init(void);

void MPU6050_Proc(void);

void MPU6050_GetResult(float *pAccel, float *pTemp, float *pGyro, float *pAngel);













#endif //TTEST_APP_MPU6050_H
