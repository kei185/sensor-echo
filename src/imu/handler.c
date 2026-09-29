#include <stdbool.h>
#include <stdint.h>
#include "main.h"
#include "stm32f4xx_hal_def.h"
#include "stm32f4xx_hal_spi.h"
#include "imu/sys.h"
#include "imu/handler.h"
#include "imu/frame.h"

bool imu_accel_ready;
bool imu_rot_ready;

ImuRxBuf rx_buf = {
        .rot =
                {
                        .x = 0,
                        .y = 0,
                        .z = 0,
                },
        .trans =
                {
                        .x = 0,
                        .y = 0,
                        .z = 0,
                },
};

const ImuRxBuf* const IMU_RX_BUF = &rx_buf;

const Imu imu;

void imu_setup()
{
        imu_accel_ready = imu_rot_ready = false;
        //
}

void imu_handler()
{
        uint8_t tx_buf[1] = {(uint8_t)(RW_READ << 7) | IMU_REG_OUTX_L_G};

        HAL_SPI_Transmit(&hspi2, tx_buf, 1, HAL_MAX_DELAY);
        HAL_SPI_Receive(&hspi2, (uint8_t*)&rx_buf, sizeof(ImuRxBuf), HAL_MAX_DELAY);
}