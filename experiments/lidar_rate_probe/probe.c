#include "experiments/lidar_rate_probe/probe.h"

#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <string.h>

#include "experiments/lidar_rate_probe/scan_meter.h"
#include "lidar/parser.h"
#include "lidar/sys.h"
#include "tx/frame.h"
#include "tx/scan.h"

// One measurement run:
// 1. Stop scanning and ask which frequency the LiDAR is configured to use.
// 2. Start circular RX DMA, start scanning, and discard the first second.
// 3. Count bytes and points for five seconds. Convert each complete packet
//    into a throwaway PC frame and measure the conversion time.
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
// Every converted frame replaces the previous one. No PC DMA transfer occurs.
static uint8_t tx_trash[TX_FRAME_HEADER_SIZE + 255u * sizeof(ParserScannedPoint)];
static volatile uint8_t output_guard;

typedef struct
{
        uint64_t total_cycles;
        uint32_t max_frame_cycles;
        uint32_t converted_frames;
        uint32_t converted_points;
        uint32_t failed_frames;
        uint32_t max_unread_bytes;
} ConversionStats;

static bool start_cycle_counter(void)
{
        // DWT counts Cortex-M4 clock cycles. Start it before the timed sample.
        CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;
        DWT->CYCCNT = 0u;
        DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
        return (DWT->CTRL & DWT_CTRL_CYCCNTENA_Msk) != 0u;
}

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
drain_ring(
        DMA_HandleTypeDef* dma,
        uint16_t*          read_position,
        LidarScanMeter*    meter,
        ConversionStats*  conversion)
{
        // Take one snapshot of DMA's write position, then consume every new
        // byte up to that position. read_position persists across polls.
        uint16_t write_position = dma_write_position(dma);
        // Keep the DMA position read ordered before the following buffer reads.
        __DMB();
        if (meter != NULL) {
                uint32_t unread =
                        (write_position + PROBE_RX_RING_SIZE - *read_position) %
                        PROBE_RX_RING_SIZE;
                if (unread > conversion->max_unread_bytes)
                        conversion->max_unread_bytes = unread;
        }
        while (*read_position != write_position) {
                // During warm-up we still drain the ring, but do not count it.
                if (meter != NULL &&
                    lidar_scan_meter_feed(meter, rx_ring[*read_position])) {
                        // Time the same scan converter used by normal TX code.
                        // Only the destination is different: the next packet
                        // overwrites this scratch frame.
                        uint32_t start = DWT->CYCCNT;
                        size_t frame_len = tx_scan_frame_write(
                                meter->packet,
                                meter->packet_length,
                                tx_trash,
                                sizeof(tx_trash),
                                HAL_GetTick());
                        uint32_t cycles = DWT->CYCCNT - start;
                        if (frame_len == 0u) {
                                ++conversion->failed_frames;
                        } else {
                                ++conversion->converted_frames;
                                conversion->converted_points += meter->lsn;
                                conversion->total_cycles += cycles;
                                if (cycles > conversion->max_frame_cycles)
                                        conversion->max_frame_cycles = cycles;
                                // Read the output so the compiler must keep
                                // the conversion even in optimized builds.
                                output_guard ^= tx_trash[frame_len - 1u];
                        }
                }
                *read_position = (uint16_t)((*read_position + 1u) % PROBE_RX_RING_SIZE);
        }
}

static void print_result(
        UART_HandleTypeDef*   pc,
        const LidarScanMeter* meter,
        const ConversionStats* conversion,
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
                     meter->complete_laps >= 2u && meter->malformed_packets == 0u &&
                     conversion->failed_frames == 0u &&
                     conversion->converted_frames == meter->packets;

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

static void print_conversion_result(
        UART_HandleTypeDef*    pc,
        const ConversionStats* conversion,
        uint32_t               core_hz)
{
        char line[192];
        uint32_t cycles_per_point =
                conversion->converted_points == 0u
                        ? 0u
                        : (uint32_t)(conversion->total_cycles /
                                     conversion->converted_points);
        uint32_t max_frame_us =
                core_hz == 0u
                        ? 0u
                        : (uint32_t)((uint64_t)conversion->max_frame_cycles * 1000000u /
                                     core_hz);
        int written = snprintf(
                line,
                sizeof(line),
                "[CONVERT PROBE] frames=%lu failed=%lu cycles/point=%lu "
                "max_frame_cycles=%lu max_frame_us=%lu max_unread=%lu core_hz=%lu\r\n",
                (unsigned long)conversion->converted_frames,
                (unsigned long)conversion->failed_frames,
                (unsigned long)cycles_per_point,
                (unsigned long)conversion->max_frame_cycles,
                (unsigned long)max_frame_us,
                (unsigned long)conversion->max_unread_bytes,
                (unsigned long)core_hz);
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

        if (!start_cycle_counter()) {
                send_lidar_command(lidar, MSG_TYPE_STOP);
                HAL_UART_DMAStop(lidar);
                send_line(pc, "[SCAN PROBE] CPU cycle counter unavailable.\r\n");
                return;
        }

        uint16_t       read_position   = 0u;
        uint32_t       warmup_start    = HAL_GetTick();
        uint32_t       previous_poll   = warmup_start;
        uint32_t       max_poll_gap_ms = 0u;
        static LidarScanMeter meter;
        ConversionStats        conversion = {0};

        // Let the motor settle for one second. Keep draining DMA so the ring
        // cannot fill while those startup bytes are deliberately ignored.
        while (HAL_GetTick() - warmup_start < PROBE_WARMUP_MS) {
                uint32_t now = HAL_GetTick();
                uint32_t gap = now - previous_poll;
                if (gap > max_poll_gap_ms)
                        max_poll_gap_ms = gap;
                previous_poll = now;
                drain_ring(lidar->hdmarx, &read_position, NULL, NULL);
        }

        drain_ring(lidar->hdmarx, &read_position, NULL, NULL);
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
                drain_ring(lidar->hdmarx, &read_position, &meter, &conversion);
        }
        drain_ring(lidar->hdmarx, &read_position, &meter, &conversion);
        uint32_t duration_ms = HAL_GetTick() - sample_start;
        uint32_t uart_error  = lidar->ErrorCode;

        // Finish the timed sample before stopping the sensor or printing.
        send_lidar_command(lidar, MSG_TYPE_STOP);
        HAL_Delay(50u);
        HAL_UART_DMAStop(lidar);
        print_result(pc, &meter, &conversion, duration_ms, max_poll_gap_ms, uart_error);
        print_conversion_result(pc, &conversion, HAL_RCC_GetHCLKFreq());
}
