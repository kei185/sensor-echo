#ifndef IMU_SYS_H
#define IMU_SYS_H

/**
 * @brief LSM6DSV16X primary-interface register addresses.
 *
 * Values are 7-bit addresses and do not include the SPI read/write command bit.
 * Reserved addresses are intentionally omitted.
 */
typedef enum
{
        IMU_REG_FUNC_CFG_ACCESS = 0x01,
        IMU_REG_PIN_CTRL        = 0x02,
        IMU_REG_IF_CFG          = 0x03,
        IMU_REG_ODR_TRIG_CFG    = 0x06,

        IMU_REG_FIFO_CTRL1       = 0x07,
        IMU_REG_FIFO_CTRL2       = 0x08,
        IMU_REG_FIFO_CTRL3       = 0x09,
        IMU_REG_FIFO_CTRL4       = 0x0A,
        IMU_REG_COUNTER_BDR_REG1 = 0x0B,
        IMU_REG_COUNTER_BDR_REG2 = 0x0C,

        IMU_REG_INT1_CTRL = 0x0D,
        IMU_REG_INT2_CTRL = 0x0E,
        IMU_REG_WHO_AM_I  = 0x0F,

        IMU_REG_CTRL1       = 0x10,
        IMU_REG_CTRL2       = 0x11,
        IMU_REG_CTRL3       = 0x12,
        IMU_REG_CTRL4       = 0x13,
        IMU_REG_CTRL5       = 0x14,
        IMU_REG_CTRL6       = 0x15,
        IMU_REG_CTRL7       = 0x16,
        IMU_REG_CTRL8       = 0x17,
        IMU_REG_CTRL9       = 0x18,
        IMU_REG_CTRL10      = 0x19,
        IMU_REG_CTRL_STATUS = 0x1A,

        IMU_REG_FIFO_STATUS1 = 0x1B,
        IMU_REG_FIFO_STATUS2 = 0x1C,
        IMU_REG_ALL_INT_SRC  = 0x1D,
        IMU_REG_STATUS_REG   = 0x1E,

        IMU_REG_OUT_TEMP_L = 0x20,
        IMU_REG_OUT_TEMP_H = 0x21,
        IMU_REG_OUTX_L_G   = 0x22,
        IMU_REG_OUTX_H_G   = 0x23,
        IMU_REG_OUTY_L_G   = 0x24,
        IMU_REG_OUTY_H_G   = 0x25,
        IMU_REG_OUTZ_L_G   = 0x26,
        IMU_REG_OUTZ_H_G   = 0x27,
        IMU_REG_OUTX_L_A   = 0x28,
        IMU_REG_OUTX_H_A   = 0x29,
        IMU_REG_OUTY_L_A   = 0x2A,
        IMU_REG_OUTY_H_A   = 0x2B,
        IMU_REG_OUTZ_L_A   = 0x2C,
        IMU_REG_OUTZ_H_A   = 0x2D,

        IMU_REG_UI_OUTX_L_G_OIS_EIS = 0x2E,
        IMU_REG_UI_OUTX_H_G_OIS_EIS = 0x2F,
        IMU_REG_UI_OUTY_L_G_OIS_EIS = 0x30,
        IMU_REG_UI_OUTY_H_G_OIS_EIS = 0x31,
        IMU_REG_UI_OUTZ_L_G_OIS_EIS = 0x32,
        IMU_REG_UI_OUTZ_H_G_OIS_EIS = 0x33,

        IMU_REG_UI_OUTX_L_A_OIS_DUALC = 0x34,
        IMU_REG_UI_OUTX_H_A_OIS_DUALC = 0x35,
        IMU_REG_UI_OUTY_L_A_OIS_DUALC = 0x36,
        IMU_REG_UI_OUTY_H_A_OIS_DUALC = 0x37,
        IMU_REG_UI_OUTZ_L_A_OIS_DUALC = 0x38,
        IMU_REG_UI_OUTZ_H_A_OIS_DUALC = 0x39,
        IMU_REG_AH_QVAR_OUT_L         = 0x3A,
        IMU_REG_AH_QVAR_OUT_H         = 0x3B,

        IMU_REG_TIMESTAMP0               = 0x40,
        IMU_REG_TIMESTAMP1               = 0x41,
        IMU_REG_TIMESTAMP2               = 0x42,
        IMU_REG_TIMESTAMP3               = 0x43,
        IMU_REG_UI_STATUS_REG_OIS        = 0x44,
        IMU_REG_WAKE_UP_SRC              = 0x45,
        IMU_REG_TAP_SRC                  = 0x46,
        IMU_REG_D6D_SRC                  = 0x47,
        IMU_REG_STATUS_MASTER_MAINPAGE   = 0x48,
        IMU_REG_EMB_FUNC_STATUS_MAINPAGE = 0x49,
        IMU_REG_FSM_STATUS_MAINPAGE      = 0x4A,
        IMU_REG_MLC_STATUS_MAINPAGE      = 0x4B,
        IMU_REG_INTERNAL_FREQ_FINE       = 0x4F,

        IMU_REG_FUNCTIONS_ENABLE = 0x50,
        IMU_REG_DEN              = 0x51,
        IMU_REG_INACTIVITY_DUR   = 0x54,
        IMU_REG_INACTIVITY_THS   = 0x55,
        IMU_REG_TAP_CFG0         = 0x56,
        IMU_REG_TAP_CFG1         = 0x57,
        IMU_REG_TAP_CFG2         = 0x58,
        IMU_REG_TAP_THS_6D       = 0x59,
        IMU_REG_TAP_DUR          = 0x5A,
        IMU_REG_WAKE_UP_THS      = 0x5B,
        IMU_REG_WAKE_UP_DUR      = 0x5C,
        IMU_REG_FREE_FALL        = 0x5D,
        IMU_REG_MD1_CFG          = 0x5E,
        IMU_REG_MD2_CFG          = 0x5F,

        IMU_REG_HAODR_CFG         = 0x62,
        IMU_REG_EMB_FUNC_CFG      = 0x63,
        IMU_REG_UI_HANDSHAKE_CTRL = 0x64,
        IMU_REG_UI_SPI2_SHARED_0  = 0x65,
        IMU_REG_UI_SPI2_SHARED_1  = 0x66,
        IMU_REG_UI_SPI2_SHARED_2  = 0x67,
        IMU_REG_UI_SPI2_SHARED_3  = 0x68,
        IMU_REG_UI_SPI2_SHARED_4  = 0x69,
        IMU_REG_UI_SPI2_SHARED_5  = 0x6A,
        IMU_REG_CTRL_EIS          = 0x6B,

        IMU_REG_UI_INT_OIS   = 0x6F,
        IMU_REG_UI_CTRL1_OIS = 0x70,
        IMU_REG_UI_CTRL2_OIS = 0x71,
        IMU_REG_UI_CTRL3_OIS = 0x72,
        IMU_REG_X_OFS_USR    = 0x73,
        IMU_REG_Y_OFS_USR    = 0x74,
        IMU_REG_Z_OFS_USR    = 0x75,

        IMU_REG_FIFO_DATA_OUT_TAG = 0x78,
        IMU_REG_FIFO_DATA_OUT_X_L = 0x79,
        IMU_REG_FIFO_DATA_OUT_X_H = 0x7A,
        IMU_REG_FIFO_DATA_OUT_Y_L = 0x7B,
        IMU_REG_FIFO_DATA_OUT_Y_H = 0x7C,
        IMU_REG_FIFO_DATA_OUT_Z_L = 0x7D,
        IMU_REG_FIFO_DATA_OUT_Z_H = 0x7E,
} ImuRegisterAddress;

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

#include <stdint.h>
typedef struct
{
        uint8_t CTRL1;
        uint8_t CTRL2;
        uint8_t CTRL3;
        uint8_t INT1_CTRL;
        uint8_t INT2_CTRL;
        uint8_t IF_CFG;
} Imu;

#endif /* IMU_SYS_H */
