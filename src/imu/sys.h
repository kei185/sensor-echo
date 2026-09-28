#ifndef IMU_SYS_H
#define IMU_SYS_H

/**
 * @brief Accelerometer operating modes for CTRL1 OP_MODE_XL[2:0].
 *
 * Values are already placed in register bits [6:4].
 */
typedef enum
{
        IMU_ACCEL_MODE_HIGH_PERFORMANCE  = 0x00,
        IMU_ACCEL_MODE_HIGH_ACCURACY_ODR = 0x10,
        IMU_ACCEL_MODE_ODR_TRIGGERED     = 0x30,
        IMU_ACCEL_MODE_LOW_POWER_1       = 0x40,
        IMU_ACCEL_MODE_LOW_POWER_2       = 0x50,
        IMU_ACCEL_MODE_LOW_POWER_3       = 0x60,
        IMU_ACCEL_MODE_NORMAL            = 0x70,
} ImuAccelMode;

/**
 * @brief Accelerometer output data rates for CTRL1 ODR_XL[3:0].
 */
typedef enum
{
        IMU_ACCEL_ODR_POWER_DOWN = 0x00,
        IMU_ACCEL_ODR_1_875_HZ   = 0x01,
        IMU_ACCEL_ODR_7_5_HZ     = 0x02,
        IMU_ACCEL_ODR_15_HZ      = 0x03,
        IMU_ACCEL_ODR_30_HZ      = 0x04,
        IMU_ACCEL_ODR_60_HZ      = 0x05,
        IMU_ACCEL_ODR_120_HZ     = 0x06,
        IMU_ACCEL_ODR_240_HZ     = 0x07,
        IMU_ACCEL_ODR_480_HZ     = 0x08,
        IMU_ACCEL_ODR_960_HZ     = 0x09,
        IMU_ACCEL_ODR_1_92_KHZ   = 0x0A,
        IMU_ACCEL_ODR_3_84_KHZ   = 0x0B,
        IMU_ACCEL_ODR_7_68_KHZ   = 0x0C,
} ImuAccelOdr;

/**
 * @brief Gyroscope operating modes for CTRL2 OP_MODE_G[2:0].
 *
 * Values are already placed in register bits [6:4].
 */
typedef enum
{
        IMU_GYRO_MODE_HIGH_PERFORMANCE  = 0x00,
        IMU_GYRO_MODE_HIGH_ACCURACY_ODR = 0x10,
        IMU_GYRO_MODE_ODR_TRIGGERED     = 0x30,
        IMU_GYRO_MODE_SLEEP             = 0x40,
        IMU_GYRO_MODE_LOW_POWER         = 0x50,
} ImuGyroMode;

/**
 * @brief Gyroscope output data rates for CTRL2 ODR_G[3:0].
 */
typedef enum
{
        IMU_GYRO_ODR_POWER_DOWN = 0x00,
        IMU_GYRO_ODR_7_5_HZ     = 0x02,
        IMU_GYRO_ODR_15_HZ      = 0x03,
        IMU_GYRO_ODR_30_HZ      = 0x04,
        IMU_GYRO_ODR_60_HZ      = 0x05,
        IMU_GYRO_ODR_120_HZ     = 0x06,
        IMU_GYRO_ODR_240_HZ     = 0x07,
        IMU_GYRO_ODR_480_HZ     = 0x08,
        IMU_GYRO_ODR_960_HZ     = 0x09,
        IMU_GYRO_ODR_1_92_KHZ   = 0x0A,
        IMU_GYRO_ODR_3_84_KHZ   = 0x0B,
        IMU_GYRO_ODR_7_68_KHZ   = 0x0C,
} ImuGyroOdr;

#endif /* IMU_SYS_H */
