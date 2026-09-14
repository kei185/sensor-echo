#include "experiments/lidar_rate_probe/probe.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "experiments/lidar_rate_probe/scan_meter.h"
#include "lidar/sys.h"

#define PROBE_RX_RING_SIZE 4096u
#define PROBE_WARMUP_MS    1000u
#define PROBE_SAMPLE_MS    5000u
// At 230400 baud, 4096 bytes need at least 177 ms on the wire.
#define PROBE_MAX_POLL_GAP_MS 100u

static uint8_t rx_ring[PROBE_RX_RING_SIZE];

static void send_line(UART_HandleTypeDef* pc, const char* line)
{
        HAL_UART_Transmit(pc, (const uint8_t*)line, (uint16_t)strlen(line), 1000u);
}

static bool send_lidar_command(UART_HandleTypeDef* lidar, MsgType command)
{
        return HAL_UART_Transmit(lidar, MSG[command], MSG_SIZE, 200u) == HAL_OK;
}

static bool read_scan_frequency(UART_HandleTypeDef* lidar, uint32_t* centihertz)
{
        uint8_t  previous = 0u;
        uint32_t start    = HAL_GetTick();

        __HAL_UART_CLEAR_OREFLAG(lidar);
        if (!send_lidar_command(lidar, MSG_TYPE_RX_FREQ))
                return false;

        while (HAL_GetTick() - start < 1000u) {
                uint8_t byte = 0u;
                if (HAL_UART_Receive(lidar, &byte, 1u, 50u) != HAL_OK)
                        continue;

                if (previous == 0xa5u && byte == 0x5au) {
                        uint8_t rest[9];
                        if (HAL_UART_Receive(lidar, rest, sizeof(rest), 200u) != HAL_OK)
                                return false;
                        // Single response: four content bytes, type 0x04.
                        if (rest[0] != 4u || rest[1] != 0u || rest[2] != 0u ||
                            rest[3] != 0u || rest[4] != 0x04u) {
                                previous = 0u;
                                continue;
                        }
                        *centihertz = (uint32_t)rest[5] | ((uint32_t)rest[6] << 8) |
                                      ((uint32_t)rest[7] << 16) |
                                      ((uint32_t)rest[8] << 24);
                        return true;
                }
                previous = byte;
        }

        return false;
}

static bool configure_byte_circular_dma(UART_HandleTypeDef* lidar)
{
        DMA_HandleTypeDef* dma = lidar->hdmarx;
        if (dma == NULL || HAL_DMA_DeInit(dma) != HAL_OK)
                return false;

        dma->Init.PeriphDataAlignment = DMA_PDATAALIGN_BYTE;
        dma->Init.MemDataAlignment    = DMA_MDATAALIGN_BYTE;
        dma->Init.Mode                = DMA_CIRCULAR;
        dma->Init.FIFOMode            = DMA_FIFOMODE_DISABLE;
        dma->Init.MemBurst            = DMA_MBURST_SINGLE;
        dma->Init.PeriphBurst         = DMA_PBURST_SINGLE;
        return HAL_DMA_Init(dma) == HAL_OK;
}

static uint16_t dma_write_position(DMA_HandleTypeDef* dma)
{
        uint32_t remaining = __HAL_DMA_GET_COUNTER(dma);
        return (uint16_t)((PROBE_RX_RING_SIZE - remaining) % PROBE_RX_RING_SIZE);
}

static void
drain_ring(DMA_HandleTypeDef* dma, uint16_t* read_position, LidarScanMeter* meter)
{
        uint16_t write_position = dma_write_position(dma);
        __DMB();
        while (*read_position != write_position) {
                if (meter != NULL)
                        lidar_scan_meter_feed(meter, rx_ring[*read_position]);
                *read_position = (uint16_t)((*read_position + 1u) % PROBE_RX_RING_SIZE);
        }
}

static void print_result(
        UART_HandleTypeDef*   pc,
        const LidarScanMeter* meter,
        uint32_t              duration_ms,
        uint32_t              max_poll_gap_ms,
        uint32_t              uart_error)
{
        char     line[240];
        uint32_t bytes_per_second =
                (uint32_t)((uint64_t)meter->bytes * 1000u / duration_ms);
        uint32_t points_per_second =
                (uint32_t)((uint64_t)meter->points * 1000u / duration_ms);
        uint32_t average_points_per_lap =
                meter->complete_laps == 0u
                        ? 0u
                        : meter->complete_lap_points / meter->complete_laps;
        bool valid = uart_error == HAL_UART_ERROR_NONE &&
                     max_poll_gap_ms < PROBE_MAX_POLL_GAP_MS &&
                     meter->complete_laps >= 2u && meter->malformed_packets == 0u;

        int written = snprintf(
                line,
                sizeof(line),
                "[SCAN PROBE] %s bytes=%lu duration_ms=%lu bytes/s=%lu "
                "points/s=%lu points/lap=%lu "
                "range=%lu..%lu laps=%lu packets=%lu malformed=%lu "
                "max_poll_gap_ms=%lu uart_error=0x%lX\r\n",
                valid ? "OK" : "CHECK",
                (unsigned long)meter->bytes,
                (unsigned long)duration_ms,
                (unsigned long)bytes_per_second,
                (unsigned long)points_per_second,
                (unsigned long)average_points_per_lap,
                (unsigned long)meter->min_lap_points,
                (unsigned long)meter->max_lap_points,
                (unsigned long)meter->complete_laps,
                (unsigned long)meter->packets,
                (unsigned long)meter->malformed_packets,
                (unsigned long)max_poll_gap_ms,
                (unsigned long)uart_error);
        if (written > 0 && (size_t)written < sizeof(line))
                send_line(pc, line);
}

void lidar_rate_probe_run(UART_HandleTypeDef* lidar, UART_HandleTypeDef* pc)
{
        char     line[96];
        uint32_t centihertz = 0u;

        send_line(pc, "[SCAN PROBE] Stopping LiDAR before frequency query.\r\n");
        if (!send_lidar_command(lidar, MSG_TYPE_STOP)) {
                send_line(pc, "[SCAN PROBE] STOP command failed.\r\n");
                return;
        }
        HAL_Delay(200u);

        if (read_scan_frequency(lidar, &centihertz)) {
                int written = snprintf(
                        line,
                        sizeof(line),
                        "[SCAN PROBE] Configured scan frequency: %lu.%02lu Hz.\r\n",
                        (unsigned long)(centihertz / 100u),
                        (unsigned long)(centihertz % 100u));
                if (written > 0 && (size_t)written < sizeof(line))
                        send_line(pc, line);
        } else {
                send_line(
                        pc,
                        "[SCAN PROBE] Frequency query failed; measuring current "
                        "setting.\r\n");
        }

        if (!configure_byte_circular_dma(lidar) ||
            HAL_UART_Receive_DMA(lidar, rx_ring, PROBE_RX_RING_SIZE) != HAL_OK) {
                send_line(pc, "[SCAN PROBE] RX DMA setup failed.\r\n");
                return;
        }

        if (!send_lidar_command(lidar, MSG_TYPE_SCAN)) {
                HAL_UART_DMAStop(lidar);
                send_line(pc, "[SCAN PROBE] SCAN command failed.\r\n");
                return;
        }

        uint16_t       read_position   = 0u;
        uint32_t       warmup_start    = HAL_GetTick();
        uint32_t       previous_poll   = warmup_start;
        uint32_t       max_poll_gap_ms = 0u;
        LidarScanMeter meter;

        while (HAL_GetTick() - warmup_start < PROBE_WARMUP_MS) {
                uint32_t now = HAL_GetTick();
                uint32_t gap = now - previous_poll;
                if (gap > max_poll_gap_ms)
                        max_poll_gap_ms = gap;
                previous_poll = now;
                drain_ring(lidar->hdmarx, &read_position, NULL);
        }

        drain_ring(lidar->hdmarx, &read_position, NULL);
        lidar_scan_meter_reset(&meter);
        uint32_t sample_start = HAL_GetTick();
        previous_poll         = sample_start;
        while (HAL_GetTick() - sample_start < PROBE_SAMPLE_MS) {
                uint32_t now = HAL_GetTick();
                uint32_t gap = now - previous_poll;
                if (gap > max_poll_gap_ms)
                        max_poll_gap_ms = gap;
                previous_poll = now;
                drain_ring(lidar->hdmarx, &read_position, &meter);
        }
        drain_ring(lidar->hdmarx, &read_position, &meter);
        uint32_t duration_ms = HAL_GetTick() - sample_start;
        uint32_t uart_error  = lidar->ErrorCode;

        send_lidar_command(lidar, MSG_TYPE_STOP);
        HAL_Delay(50u);
        HAL_UART_DMAStop(lidar);
        print_result(pc, &meter, duration_ms, max_poll_gap_ms, uart_error);
}
