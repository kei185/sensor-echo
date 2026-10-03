#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>
#include <string.h>
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_gpio.h"
#include "stm32f4xx_hal_spi.h"
#include "imu/sys.h"
#include "imu/handler.h"
#include "imu/frame.h"

bool volatile imu_accel_ready;

bool volatile imu_rot_ready;

// ImuRxBuf rx_buf = {
//         .rot =
//                 {
//                         .x = 0,
//                         .y = 0,
//                         .z = 0,
//                 },
//         .trans =
//                 {
//                         .x = 0,
//                         .y = 0,
//                         .z = 0,
//                 },
// };

// const ImuRxBuf* const IMU_RX_BUF = &rx_buf;

const Imu imu = {
        .IF_CFG    = IF_CFG_DEFAULT,
        .INT1_CTRL = INT1_ACCEL_DATA_READY,
        .INT2_CTRL = INT2_GYRO_DATA_READY,
        .CTRL1     = CTRL1_DEFAULT | ACCEL_ODR_15_HZ,
        .CTRL2     = CTRL2_DEFAULT | GYRO_ODR_15_HZ,
        .CTRL3     = CTRL3_DEFAULT,
};

bool imu_setup()
{
        imu_accel_ready = imu_rot_ready = false;

        HAL_StatusTypeDef ok;
        uint8_t           tx_buf[4] = {0};

        // set IF_CFG
        tx_buf[0] = REG_IF_CFG | (RW_WRITE << 7);
        tx_buf[1] = imu.IF_CFG;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
        ok = HAL_SPI_Transmit(&hspi2, tx_buf, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

        if (ok != HAL_OK)
                return false;

        // set INTx_CTRL
        tx_buf[0] = REG_INT1_CTRL | (RW_WRITE << 7);
        tx_buf[1] = imu.INT1_CTRL;
        tx_buf[2] = imu.INT2_CTRL;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
        ok = HAL_SPI_Transmit(&hspi2, tx_buf, 3, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
        if (ok != HAL_OK)
                return false;

        // set CRTLx
        tx_buf[0] = REG_CTRL1 | (RW_WRITE << 7);
        tx_buf[1] = imu.CTRL1;
        tx_buf[2] = imu.CTRL2;
        tx_buf[3] = imu.CTRL3;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
        ok = HAL_SPI_Transmit(&hspi2, tx_buf, 4, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);
        if (ok != HAL_OK)
                return false;

        uint8_t whoami[2] = {0};
        tx_buf[0]         = REG_WHO_AM_I | (RW_READ << 7);
        tx_buf[1]         = 0;
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
        ok = HAL_SPI_TransmitReceive(&hspi2, tx_buf, whoami, 2, HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

        return whoami[1] == 0x70;
}

#define _TRX_BUF_SIZE 1 + IMU_BUF_SIZE
bool imu_handler(uint8_t* _buf)
{
        HAL_StatusTypeDef ok;

        uint8_t tx_buf[_TRX_BUF_SIZE] = {0};
        uint8_t rx_buf[_TRX_BUF_SIZE] = {0};
        tx_buf[0]                     = (uint8_t)(RW_READ << 7) | REG_OUTX_L_G;

        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 0);
        ok = HAL_SPI_TransmitReceive(
                &hspi2,
                tx_buf,
                rx_buf,
                _TRX_BUF_SIZE,
                HAL_MAX_DELAY);
        HAL_GPIO_WritePin(SPI2_CS_GPIO_Port, SPI2_CS_Pin, 1);

        if (ok != HAL_OK)
                return false;

        memcpy(_buf, rx_buf + 1, IMU_BUF_SIZE);

        return true;
}
