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
        REG_FUNC_CFG_ACCESS = 0x01,
        REG_PIN_CTRL        = 0x02,
        REG_IF_CFG          = 0x03,
        REG_ODR_TRIG_CFG    = 0x06,

        REG_FIFO_CTRL1       = 0x07,
        REG_FIFO_CTRL2       = 0x08,
        REG_FIFO_CTRL3       = 0x09,
        REG_FIFO_CTRL4       = 0x0A,
        REG_COUNTER_BDR_REG1 = 0x0B,
        REG_COUNTER_BDR_REG2 = 0x0C,

        REG_INT1_CTRL = 0x0D,
        REG_INT2_CTRL = 0x0E,
        REG_WHO_AM_I  = 0x0F,

        REG_CTRL1       = 0x10,
        REG_CTRL2       = 0x11,
        REG_CTRL3       = 0x12,
        REG_CTRL4       = 0x13,
        REG_CTRL5       = 0x14,
        REG_CTRL6       = 0x15,
        REG_CTRL7       = 0x16,
        REG_CTRL8       = 0x17,
        REG_CTRL9       = 0x18,
        REG_CTRL10      = 0x19,
        REG_CTRL_STATUS = 0x1A,

        REG_FIFO_STATUS1 = 0x1B,
        REG_FIFO_STATUS2 = 0x1C,
        REG_ALL_INT_SRC  = 0x1D,
        REG_STATUS_REG   = 0x1E,

        REG_OUT_TEMP_L = 0x20,
        REG_OUT_TEMP_H = 0x21,
        REG_OUTX_L_G   = 0x22,
        REG_OUTX_H_G   = 0x23,
        REG_OUTY_L_G   = 0x24,
        REG_OUTY_H_G   = 0x25,
        REG_OUTZ_L_G   = 0x26,
        REG_OUTZ_H_G   = 0x27,
        REG_OUTX_L_A   = 0x28,
        REG_OUTX_H_A   = 0x29,
        REG_OUTY_L_A   = 0x2A,
        REG_OUTY_H_A   = 0x2B,
        REG_OUTZ_L_A   = 0x2C,
        REG_OUTZ_H_A   = 0x2D,

        REG_UI_OUTX_L_G_OIS_EIS = 0x2E,
        REG_UI_OUTX_H_G_OIS_EIS = 0x2F,
        REG_UI_OUTY_L_G_OIS_EIS = 0x30,
        REG_UI_OUTY_H_G_OIS_EIS = 0x31,
        REG_UI_OUTZ_L_G_OIS_EIS = 0x32,
        REG_UI_OUTZ_H_G_OIS_EIS = 0x33,

        REG_UI_OUTX_L_A_OIS_DUALC = 0x34,
        REG_UI_OUTX_H_A_OIS_DUALC = 0x35,
        REG_UI_OUTY_L_A_OIS_DUALC = 0x36,
        REG_UI_OUTY_H_A_OIS_DUALC = 0x37,
        REG_UI_OUTZ_L_A_OIS_DUALC = 0x38,
        REG_UI_OUTZ_H_A_OIS_DUALC = 0x39,
        REG_AH_QVAR_OUT_L         = 0x3A,
        REG_AH_QVAR_OUT_H         = 0x3B,

        REG_TIMESTAMP0               = 0x40,
        REG_TIMESTAMP1               = 0x41,
        REG_TIMESTAMP2               = 0x42,
        REG_TIMESTAMP3               = 0x43,
        REG_UI_STATUS_REG_OIS        = 0x44,
        REG_WAKE_UP_SRC              = 0x45,
        REG_TAP_SRC                  = 0x46,
        REG_D6D_SRC                  = 0x47,
        REG_STATUS_MASTER_MAINPAGE   = 0x48,
        REG_EMB_FUNC_STATUS_MAINPAGE = 0x49,
        REG_FSM_STATUS_MAINPAGE      = 0x4A,
        REG_MLC_STATUS_MAINPAGE      = 0x4B,
        REG_INTERNAL_FREQ_FINE       = 0x4F,

        REG_FUNCTIONS_ENABLE = 0x50,
        REG_DEN              = 0x51,
        REG_INACTIVITY_DUR   = 0x54,
        REG_INACTIVITY_THS   = 0x55,
        REG_TAP_CFG0         = 0x56,
        REG_TAP_CFG1         = 0x57,
        REG_TAP_CFG2         = 0x58,
        REG_TAP_THS_6D       = 0x59,
        REG_TAP_DUR          = 0x5A,
        REG_WAKE_UP_THS      = 0x5B,
        REG_WAKE_UP_DUR      = 0x5C,
        REG_FREE_FALL        = 0x5D,
        REG_MD1_CFG          = 0x5E,
        REG_MD2_CFG          = 0x5F,

        REG_HAODR_CFG         = 0x62,
        REG_EMB_FUNC_CFG      = 0x63,
        REG_UI_HANDSHAKE_CTRL = 0x64,
        REG_UI_SPI2_SHARED_0  = 0x65,
        REG_UI_SPI2_SHARED_1  = 0x66,
        REG_UI_SPI2_SHARED_2  = 0x67,
        REG_UI_SPI2_SHARED_3  = 0x68,
        REG_UI_SPI2_SHARED_4  = 0x69,
        REG_UI_SPI2_SHARED_5  = 0x6A,
        REG_CTRL_EIS          = 0x6B,

        REG_UI_INT_OIS   = 0x6F,
        REG_UI_CTRL1_OIS = 0x70,
        REG_UI_CTRL2_OIS = 0x71,
        REG_UI_CTRL3_OIS = 0x72,
        REG_X_OFS_USR    = 0x73,
        REG_Y_OFS_USR    = 0x74,
        REG_Z_OFS_USR    = 0x75,

        REG_FIFO_DATA_OUT_TAG = 0x78,
        REG_FIFO_DATA_OUT_X_L = 0x79,
        REG_FIFO_DATA_OUT_X_H = 0x7A,
        REG_FIFO_DATA_OUT_Y_L = 0x7B,
        REG_FIFO_DATA_OUT_Y_H = 0x7C,
        REG_FIFO_DATA_OUT_Z_L = 0x7D,
        REG_FIFO_DATA_OUT_Z_H = 0x7E,
} RegisterAddress;

#define CTRL1_DEFAULT     0x00u
#define CTRL2_DEFAULT     0x00u
#define CTRL3_DEFAULT     0x44u
#define INT1_CTRL_DEFAULT 0x00u
#define INT2_CTRL_DEFAULT 0x00u
#define IF_CFG_DEFAULT    0x00u

#define CTRL_MODE_MASK 0x70u
#define CTRL_ODR_MASK  0x0Fu

/**
 * @brief CTRL3 field values.
 *
 * Select one value for each field and combine them with bitwise OR. BOOT and
 * SW_RESET are trigger bits that clear automatically.
 */
typedef enum
{
        CTRL3_BOOT_NORMAL        = 0x00,
        CTRL3_BOOT_REBOOT        = 0x80,
        CTRL3_BDU_DISABLED       = 0x00,
        CTRL3_BDU_ENABLED        = 0x40,
        CTRL3_IF_INC_DISABLED    = 0x00,
        CTRL3_IF_INC_ENABLED     = 0x04,
        CTRL3_SW_RESET_NORMAL    = 0x00,
        CTRL3_SW_RESET_TRIGGERED = 0x01,
} Ctrl3Setting;

/**
 * @brief Signals routed to the INT1 pin by INT1_CTRL.
 *
 * Multiple routes can be combined with bitwise OR.
 */
typedef enum
{
        INT1_NONE             = 0x00,
        INT1_COUNTER_BDR      = 0x40,
        INT1_FIFO_FULL        = 0x20,
        INT1_FIFO_OVERRUN     = 0x10,
        INT1_FIFO_THRESHOLD   = 0x08,
        INT1_GYRO_DATA_READY  = 0x02,
        INT1_ACCEL_DATA_READY = 0x01,
} Int1;

/**
 * @brief Signals routed to the INT2 pin by INT2_CTRL.
 *
 * Multiple routes can be combined with bitwise OR.
 */
typedef enum
{
        INT2_NONE                  = 0x00,
        INT2_EMBEDDED_FUNCTION_END = 0x80,
        INT2_COUNTER_BDR           = 0x40,
        INT2_FIFO_FULL             = 0x20,
        INT2_FIFO_OVERRUN          = 0x10,
        INT2_FIFO_THRESHOLD        = 0x08,
        INT2_EIS_GYRO_DATA_READY   = 0x04,
        INT2_GYRO_DATA_READY       = 0x02,
        INT2_ACCEL_DATA_READY      = 0x01,
} Int2;

/**
 * @brief IF_CFG field values.
 *
 * Select one value for each field and combine them with bitwise OR.
 */
typedef enum
{
        IF_CFG_SDA_PULL_UP_DISABLED           = 0x00,
        IF_CFG_SDA_PULL_UP_ENABLED            = 0x80,
        IF_CFG_SENSOR_HUB_PULL_UP_DISABLED    = 0x00,
        IF_CFG_SENSOR_HUB_PULL_UP_ENABLED     = 0x40,
        IF_CFG_ANTI_SPIKE_PROTOCOL_CONTROLLED = 0x00,
        IF_CFG_ANTI_SPIKE_ALWAYS_ENABLED      = 0x20,
        IF_CFG_INTERRUPT_ACTIVE_HIGH          = 0x00,
        IF_CFG_INTERRUPT_ACTIVE_LOW           = 0x10,
        IF_CFG_INTERRUPT_PUSH_PULL            = 0x00,
        IF_CFG_INTERRUPT_OPEN_DRAIN           = 0x08,
        IF_CFG_SPI_4_WIRE                     = 0x00,
        IF_CFG_SPI_3_WIRE                     = 0x04,
        IF_CFG_I2C_I3C_ENABLED                = 0x00,
        IF_CFG_I2C_I3C_DISABLED               = 0x01,
} InterfaceSetting;

/**
 * @brief Accelerometer operating modes for CTRL1 OP_MODE_XL[2:0].
 *
 * Values are already placed in register bits [6:4].
 */
typedef enum
{
        ACCEL_MODE_HIGH_PERFORMANCE  = 0x00,
        ACCEL_MODE_HIGH_ACCURACY_ODR = 0x10,
        ACCEL_MODE_ODR_TRIGGERED     = 0x30,
        ACCEL_MODE_LOW_POWER_1       = 0x40,
        ACCEL_MODE_LOW_POWER_2       = 0x50,
        ACCEL_MODE_LOW_POWER_3       = 0x60,
        ACCEL_MODE_NORMAL            = 0x70,
} AccelMode;

/**
 * @brief Accelerometer output data rates for CTRL1 ODR_XL[3:0].
 */
typedef enum
{
        ACCEL_ODR_POWER_DOWN = 0x00,
        ACCEL_ODR_1_875_HZ   = 0x01,
        ACCEL_ODR_7_5_HZ     = 0x02,
        ACCEL_ODR_15_HZ      = 0x03,
        ACCEL_ODR_30_HZ      = 0x04,
        ACCEL_ODR_60_HZ      = 0x05,
        ACCEL_ODR_120_HZ     = 0x06,
        ACCEL_ODR_240_HZ     = 0x07,
        ACCEL_ODR_480_HZ     = 0x08,
        ACCEL_ODR_960_HZ     = 0x09,
        ACCEL_ODR_1_92_KHZ   = 0x0A,
        ACCEL_ODR_3_84_KHZ   = 0x0B,
        ACCEL_ODR_7_68_KHZ   = 0x0C,
} AccelOdr;

/**
 * @brief Gyroscope operating modes for CTRL2 OP_MODE_G[2:0].
 *
 * Values are already placed in register bits [6:4].
 */
typedef enum
{
        GYRO_MODE_HIGH_PERFORMANCE  = 0x00,
        GYRO_MODE_HIGH_ACCURACY_ODR = 0x10,
        GYRO_MODE_ODR_TRIGGERED     = 0x30,
        GYRO_MODE_SLEEP             = 0x40,
        GYRO_MODE_LOW_POWER         = 0x50,
} GyroMode;

/**
 * @brief Gyroscope output data rates for CTRL2 ODR_G[3:0].
 */
typedef enum
{
        GYRO_ODR_POWER_DOWN = 0x00,
        GYRO_ODR_7_5_HZ     = 0x02,
        GYRO_ODR_15_HZ      = 0x03,
        GYRO_ODR_30_HZ      = 0x04,
        GYRO_ODR_60_HZ      = 0x05,
        GYRO_ODR_120_HZ     = 0x06,
        GYRO_ODR_240_HZ     = 0x07,
        GYRO_ODR_480_HZ     = 0x08,
        GYRO_ODR_960_HZ     = 0x09,
        GYRO_ODR_1_92_KHZ   = 0x0A,
        GYRO_ODR_3_84_KHZ   = 0x0B,
        GYRO_ODR_7_68_KHZ   = 0x0C,
} GyroOdr;

#include <stdint.h>
typedef struct
{
        uint8_t IF_CFG;
        uint8_t INT1_CTRL;
        uint8_t INT2_CTRL;
        uint8_t CTRL1;
        uint8_t CTRL2;
        uint8_t CTRL3;
} Imu;

#endif /* IMU_SYS_H */
