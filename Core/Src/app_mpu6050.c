

#include "app_mpu6050.h"
#include "i2c.h"
#include "math.h"
#include "stm32f4xx_hal_i2c.h"
#include "MahonyAHRS.h"
#include <string.h>

static float ax, ay, az, gx, gy, gz, temperature, yaw, pitch, roll;
static float bias_ax = 0, bias_ay = 0, bias_az = 0;
static float bias_gx = 0, bias_gy = 0, bias_gz = 0;
uint8_t buffer[14] = {0};
static int NUM_SAMPLE = 100;
int ini_offset = 100;
char str1[100];

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

    if(ini_offset > 0){
			// Replace these lines with actual sensor data collection
		//bias_az += az;
		bias_gx += gx;
		bias_gy += gy;
		bias_gz += gz;
		ini_offset--;
    }else if(ini_offset == 0){
		//bias_az /= NUM_SAMPLE;
		bias_gx /= NUM_SAMPLE;
		bias_gy /= NUM_SAMPLE;
		bias_gz /= NUM_SAMPLE;
		ini_offset --;
    }else{
    	//az -= bias_az;
    	//az += 1;
    	gx -= bias_gx;
    	gy -= bias_gy;
    	gz -= bias_gz;

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
    //sprintf(str1, "gx = %.3f gy = %.3f gz = %.3f;\r\n", bias_gx, bias_gy, bias_gz );



  /*  // Scale gyroscope data to radians/sec
    gx = (gx - bias_gx) * (PI / 180.0f);
    gy = (gy - bias_gy) * (PI / 180.0f);
    gz = (gz - bias_gz) * (PI / 180.0f);

    // Normalize accelerometer data
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax = ax * norm;
    ay = ay * norm;
    az = az * norm;

    /*gx *= (PI / 180.0f);
    gy *= (PI / 180.0f);
    gz *= (PI / 180.0f);

       // Normalize accelerometer data
    float norm = invSqrt(ax * ax + ay * ay + az * az);
    ax *= norm;
    ay *= norm;
    az *= norm;


    // Update the Mahony filter
	MahonyAHRSupdateIMU(gx, gy, gz, ax, ay, az);

	// Calculate Euler angles from the quaternion
	roll = atan2f(2.0f * (q0 * q1 + q2 * q3), 1.0f - 2.0f * (q1 * q1 + q2 * q2));
	pitch = asinf(2.0f * (q0 * q2 - q3 * q1));
	yaw = atan2f(2.0f * (q0 * q3 + q1 * q2), 1.0f - 2.0f * (q2 * q2 + q3 * q3));

	// Convert from radians to degrees
	roll *= (180.0f / PI);
	pitch *= (180.0f / PI);
	yaw *= (180.0f / PI);*/


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

    //pBuffer = buffer;
}


