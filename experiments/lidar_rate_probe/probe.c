#include "experiments/lidar_rate_probe/probe.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "experiments/lidar_rate_probe/scan_meter.h"
#include "lidar/sys.h"

// One measurement run:
// 1. Stop scanning and ask which frequency the LiDAR is configured to use.
// 2. Start circular RX DMA, start scanning, and discard the first second.
// 3. Count incoming bytes and scan points for five seconds, then print rates.
// No frequency-setting command is sent at any point.

// DMA writes incoming UART bytes into this array and wraps at the end.
// The CPU reads them from the same array during frequent polls.
#define PROBE_RX_RING_SIZE 4096u
#define PROBE_WARMUP_MS    1000u
#define PROBE_SAMPLE_MS    5000u
// At 230400 baud (8N1), 4096 bytes need at least 177 ms on the wire.
// Flag gaps of 100 ms or more, before a full unseen wrap becomes possible.
#define PROBE_MAX_POLL_GAP_MS 100u

static uint8_t rx_ring[PROBE_RX_RING_SIZE];

static void send_line(UART_HandleTypeDef* pc, const char* line)
{
        // PC output is blocking, so we only call this outside the timed sample.
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

        // Read the configured frequency, not the measured motor speed.
        // This command does not change the frequency.
        // Clear an old overrun before trying to receive the short response.
        __HAL_UART_CLEAR_OREFLAG(lidar);
        if (!send_lidar_command(lidar, MSG_TYPE_RX_FREQ))
                return false;

        while (HAL_GetTick() - start < 1000u) {
                uint8_t byte = 0u;
                if (HAL_UART_Receive(lidar, &byte, 1u, 50u) != HAL_OK)
                        continue;

                // Find the A5 5A response marker even if stray bytes precede it.
                if (previous == 0xa5u && byte == 0x5au) {
                        uint8_t rest[9];
                        if (HAL_UART_Receive(lidar, rest, sizeof(rest), 200u) != HAL_OK)
                                return false;
                        // Descriptor 04 00 00 00 means four content bytes and
                        // single-response mode. The next byte is type 04.
                        if (rest[0] != 4u || rest[1] != 0u || rest[2] != 0u ||
                            rest[3] != 0u || rest[4] != 0x04u) {
                                previous = 0u;
                                continue;
                        }
                        // The four content bytes are frequency in 0.01 Hz,
                        // least significant byte first. 600 means 6.00 Hz.
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
        // The generated DMA settings are for normal, word-sized transfers.
        // This optional probe needs one byte per UART character and a ring
        // that keeps receiving throughout the five-second sample.
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
        // NDTR counts down from 4096 to zero. Subtracting it from 4096 gives
        // the next position DMA will write; modulo maps the end back to zero.
        uint32_t remaining = __HAL_DMA_GET_COUNTER(dma);
        return (uint16_t)((PROBE_RX_RING_SIZE - remaining) % PROBE_RX_RING_SIZE);
}

static void
drain_ring(DMA_HandleTypeDef* dma, uint16_t* read_position, LidarScanMeter* meter)
{
        // Take one snapshot of DMA's write position, then consume every new
        // byte up to that position. read_position persists across polls.
        uint16_t write_position = dma_write_position(dma);
        // Keep the DMA position read ordered before the following buffer reads.
        __DMB();
        while (*read_position != write_position) {
                // During warm-up we still drain the ring, but do not count it.
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
        char line[240];
        // Convert counts over duration_ms into per-second rates. Use 64-bit
        // multiplication so multiplying the count by 1000 cannot overflow.
        uint32_t bytes_per_second =
                (uint32_t)((uint64_t)meter->bytes * 1000u / duration_ms);
        uint32_t points_per_second =
                (uint32_t)((uint64_t)meter->points * 1000u / duration_ms);
        // Divide only points from complete laps by the number of such laps.
        uint32_t average_points_per_lap =
                meter->complete_laps == 0u
                        ? 0u
                        : meter->complete_lap_points / meter->complete_laps;
        // CHECK means the numbers may be incomplete or based on too few laps.
        // The gap check is a warning; it cannot directly detect a missed wrap.
        bool valid = uart_error == HAL_UART_ERROR_NONE &&
                     max_poll_gap_ms < PROBE_MAX_POLL_GAP_MS &&
                     meter->complete_laps >= 2u && meter->malformed_packets == 0u;

        // snprintf returns the size the line would need. Do not send a
        // truncated report, because it could hide an error field at the end.
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

        // Stop any old scan stream so its packets cannot mix with the short
        // frequency-query response. We do not send a frequency-setting command.
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

        // Start DMA before SCAN so the first response and scan bytes are not
        // lost. The packet counter will ignore non-scan response bytes.
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

        // Let the motor settle for one second. Keep draining DMA so the ring
        // cannot fill while those startup bytes are deliberately ignored.
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
        // Poll continuously for five seconds. Avoid PC prints here because
        // blocking UART output could let DMA overwrite unread ring bytes.
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

        // Finish the timed sample before stopping the sensor or printing.
        send_lidar_command(lidar, MSG_TYPE_STOP);
        HAL_Delay(50u);
        HAL_UART_DMAStop(lidar);
        print_result(pc, &meter, duration_ms, max_poll_gap_ms, uart_error);
}
