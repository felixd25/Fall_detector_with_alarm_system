//
// Created by Administrator on 2024/10/3.
//


#include "app_mpu6050.h"
#include "i2c.h"
#include "math.h"
#include "OLED.h"

static float ax, ay, az, gx, gy, gz, temperature, yaw, pitch, roll;

// I2C初始化
void i2c_init(void)
{
    HAL_I2C_Init(&hi2c2);
}


void MPU6050_Init(void)
{
    // 初始化I2C
    i2c_init();

    // 复位
    HAL_I2C_Mem_Write(&hi2c2, MPU6050, POWER_REG, I2C_MEMADD_SIZE_8BIT, RESET_BIT, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, buf, 2, HAL_MAX_DELAY);    //复位MPU6050，清空寄存器

    HAL_Delay(100);

    // 唤醒
    HAL_I2C_Mem_Write(&hi2c2, MPU6050, POWER_REG, I2C_MEMADD_SIZE_8BIT, WEAK_BIT, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, buf, 2, HAL_MAX_DELAY);    //唤醒MPU6050，退出休眠模式


    // 设置采样率为200Hz
    HAL_I2C_Mem_Write(&hi2c2, MPU6050, SAMPLE_RATE_DIVIDER_REG, I2C_MEMADD_SIZE_8BIT, HZ200, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, buf, 2, HAL_MAX_DELAY);

    // 设置加速度计量程为+-2G
    HAL_I2C_Mem_Write(&hi2c2, MPU6050, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, ACCEL_FS_SEL_2G, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, buf, 2, HAL_MAX_DELAY);

    // 设置陀螺仪满量程为+-2000dps
    HAL_I2C_Mem_Write(&hi2c2, MPU6050, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, GYRO_FS_SEL_2000DPS, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, buf, 2, HAL_MAX_DELAY);
}

//要求每隔5ms执行一次下面的函数
void MPU6050_Proc(void)
{
    HAL_Delay(5);
    // 从MPU6050读取数据
    uint8_t buffer[14];
    HAL_I2C_Mem_Read(&hi2c2, MPU6050, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, buffer, 14, HAL_MAX_DELAY);

//    HAL_I2C_Master_Transmit(&hi2c2, MPU6050, (uint8_t *)ACCEL_XOUT_H_REG, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Receive(&hi2c2, MPU6050, buffer, 14, HAL_MAX_DELAY);

    // 解析数据
    ax = (int16_t)(buffer[0] << 8 | buffer[1]) / 16384.0f;
    ay = (int16_t)(buffer[2] << 8 | buffer[3]) / 16384.0f;
    az = (int16_t)(buffer[4] << 8 | buffer[5]) / 16384.0f;
    temperature = (int16_t)(buffer[6] << 8 | buffer[7]) / 340.0f + 36.53f;
    gx = (int16_t)(buffer[8] << 8 | buffer[9]) / 16.4f;
    gy = (int16_t)(buffer[10] << 8 | buffer[11]) / 16.4f;
    gz = (int16_t)(buffer[12] << 8 | buffer[13]) / 16.4f;

//    OLED_ShowNum(0, 16, (int32_t)temperature, 5, OLED_8X16);
//    OLED_ShowNum(0, 32, (int32_t)ax, 5, OLED_8X16);
//    OLED_Update();

    //使用加速度计算欧拉角
    float pitch_a = -atan2(ax, az) * 180.0f / PI;
    float roll_a = atan2(ay, az) * 180.0f / PI;

    //使用陀螺仪计算欧拉角
    float yaw_g = yaw + gz * DT;
    float pitch_g = pitch + gy * DT;
    float roll_g = roll + gx * DT;

    //互补滤波
    yaw = yaw_g;
    pitch = ALPHA * pitch_g + (1 - ALPHA) * pitch_a;
    roll = ALPHA * roll_g + (1 - ALPHA) * roll_a;
}

void MPU6050_GetResult(float *pAccel, float *pTemp, float *pGyro, float *pAngel)
{
    pAccel[0] = ax;
    pAccel[1] = ay;
    pAccel[2] = az;

    *pTemp = temperature;

    pGyro[0] = gx;
    pGyro[1] = gy;
    pGyro[2] = gz;

    pAngel[0] = roll;
    pAngel[1] = pitch;
    pAngel[2] = yaw;
}
