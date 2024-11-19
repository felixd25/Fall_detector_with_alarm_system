

#include "app_mpu6050.h"
#include "i2c.h"
#include "math.h"
#include "OLED.h"
#include "stm32f4xx_hal_i2c.h"

static float ax, ay, az, gx, gy, gz, temperature, yaw, pitch, roll;
uint8_t buffer[14] = {0};

// I2C Initialization
void i2c_init(void)
{
	__HAL_RCC_I2C1_FORCE_RESET();
	HAL_Delay(100);
	__HAL_RCC_I2C1_RELEASE_RESET();

	HAL_I2C_DeInit(&hi2c1);
	HAL_Delay(100);
    HAL_I2C_Init(&hi2c1);
}





void MPU6050_Init(void)
{

    // Initailize I2C
    i2c_init();


    // Reset
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, POWER_REG, I2C_MEMADD_SIZE_8BIT, RESET_BIT, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, buf, 2, HAL_MAX_DELAY);    //Reset MPU6050，Clean Registers

    HAL_Delay(100);

    // Wake
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, POWER_REG, I2C_MEMADD_SIZE_8BIT, WEAK_BIT, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, buf, 2, HAL_MAX_DELAY);    //Wake MPU6050，Exit Sleep


    // Set the sample rate as 200Hz
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, SAMPLE_RATE_DIVIDER_REG, I2C_MEMADD_SIZE_8BIT, HZ200, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, buf, 2, HAL_MAX_DELAY);

    // Set accelerator range as +-2G
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, ACCEL_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, ACCEL_FS_SEL_2G, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, buf, 2, HAL_MAX_DELAY);

    // Set gyroscope range as +-2000dps
    HAL_I2C_Mem_Write(&hi2c1, MPU6050, GYRO_CONFIG_REG, I2C_MEMADD_SIZE_8BIT, GYRO_FS_SEL_2000DPS, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, buf, 2, HAL_MAX_DELAY);


}


void MPU6050_Proc(void)
{
    HAL_Delay(5);//The following actions are required to be executed every 5ms

    // Read data from MPU6050
    HAL_I2C_Mem_Read(&hi2c1, MPU6050, ACCEL_XOUT_H_REG, I2C_MEMADD_SIZE_8BIT, buffer, 14, HAL_MAX_DELAY);

//    HAL_I2C_Master_Transmit(&hi2c1, MPU6050, (uint8_t *)ACCEL_XOUT_H_REG, 1, HAL_MAX_DELAY);
//    HAL_I2C_Master_Receive(&hi2c1, MPU6050, buffer, 14, HAL_MAX_DELAY);

    // Analyze raw data from MPU6050 and assign it to integer
    ax = (int16_t)(buffer[0] << 8 | buffer[1]) / 16384.0f;
    ay = (int16_t)(buffer[2] << 8 | buffer[3]) / 16384.0f;
    az = (int16_t)(buffer[4] << 8 | buffer[5]) / 16384.0f;
    temperature = (int16_t)(buffer[6] << 8 | buffer[7]) / 340.0f + 36.53f;
    gx = (int16_t)(buffer[8] << 8 | buffer[9]) / 16.4f;
    gy = (int16_t)(buffer[10] << 8 | buffer[11]) / 16.4f;
    gz = (int16_t)(buffer[12] << 8 | buffer[13]) / 16.4f;


    //Calculate Euler Angle using Accelerator
    float pitch_a = -atan2(ax, az) * 180.0f / PI;
    float roll_a = atan2(ay, az) * 180.0f / PI;

    //Calculate Euler Angle using Gyroscope
    float yaw_g = yaw + gz * DT;
    float pitch_g = pitch + gy * DT;
    float roll_g = roll + gx * DT;

    //Complementary filtering
    yaw = yaw_g;
    pitch = ALPHA * pitch_g + (1 - ALPHA) * pitch_a;
    roll = ALPHA * roll_g + (1 - ALPHA) * roll_a;
}

void MPU6050_GetResult(float *pAccel, float *pTemp, float *pGyro, float *pAngle, uint8_t *pBuffer)
{
    pAccel[0] = ax;
    pAccel[1] = ay;
    pAccel[2] = az;

    *pTemp = temperature;

    pGyro[0] = gx;
    pGyro[1] = gy;
    pGyro[2] = gz;

    pAngle[0] = yaw;
    pAngle[1] = roll;
    pAngle[2] = pitch;

    pBuffer = buffer;
}
