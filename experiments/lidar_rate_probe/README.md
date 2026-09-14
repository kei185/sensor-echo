# LiDAR scan-rate probe

This temporary firmware mode measures the LiDAR stream at its **current** scan
frequency. It does not send a frequency-setting command. If the sensor still
uses its 6 Hz factory setting, the result describes that setting directly.

The probe uses UART4 at the firmware's configured 230400 bps for the LiDAR and
prints results on USART2 at 115200 bps. Connect to the board's USART2 serial
port, build and flash the probe, then reset the board:

```sh
cmake --preset Debug -DSENSOR_ECHO_LIDAR_RATE_PROBE=ON
cmake --build --preset Debug
```

## Measurement sequence

```mermaid
sequenceDiagram
    participant PC as PC (USART2)
    participant Board as Nucleo board
    participant LiDAR as LiDAR (UART4)
    Board->>LiDAR: STOP any previous scan
    Board->>LiDAR: Request configured scan frequency
    alt Valid frequency reply
        LiDAR-->>Board: Configured frequency
        Board-->>PC: Print configured setting
    else No valid reply
        Board-->>PC: Report query failure and continue anyway
    end
    Board->>Board: Start byte-wide circular RX DMA
    Board->>LiDAR: SCAN
    loop First 1 second: warm-up
        LiDAR-->>Board: Scan bytes
        Board->>Board: Drain DMA ring without counting
    end
    loop Next 5 seconds: measurement
        LiDAR-->>Board: Scan bytes
        Board->>Board: Count received bytes and scan points
    end
    Board->>LiDAR: STOP
    Board-->>PC: Print one measurement result
```

The frequency response is a configured setting, not a live speed measurement.
If the query fails, the five-second stream measurement still runs. The normal
startup loop runs only when the CMake option is off.

## How incoming bytes are counted

```mermaid
flowchart LR
    LiDAR[LiDAR byte stream] --> UART[UART4]
    UART --> DMA[Circular RX DMA]
    DMA --> Ring[4096-byte ring]
    DMA -. Remaining byte count .-> Position[CPU finds DMA write position]
    Ring --> Unread[CPU tracks unread positions]
    Position --> Unread
    Unread --> Phase{Measurement phase?}
    Phase -->|Warm-up| Discard[Advance read position without counting]
    Phase -->|Five-second sample| Meter[Feed unread bytes to packet counter]
    Meter --> Report[Bytes, packets, points, and complete laps]
```

DMA writes positions `0 -> 1 -> ... -> 4095 -> 0` in the same array. The CPU
keeps a separate read position and follows the DMA write position. It drains
the ring during warm-up but starts its counters only for the five-second
sample. This temporary probe ring is independent of the two-slot RX design
described for the normal firmware.

```text
[SCAN PROBE] Configured scan frequency: 6.00 Hz.
[SCAN PROBE] OK bytes=... duration_ms=... bytes/s=... points/s=... points/lap=... ...
```

`bytes/s` counts the received UART stream, including packet headers.
`points/s` and `points/lap` use each scan packet's LSN and CT start-of-lap bit;
`points/lap` averages complete laps only. A `CHECK` result indicates a UART
error, malformed packet, fewer than two complete laps, or a polling gap that
could hide a DMA ring wrap. Packet XOR is not checked in this prototype.

## Which points belong to a complete lap?

```mermaid
flowchart LR
    Partial[Partial lap at sample start]
    subgraph LapA [Complete lap A: 6 points]
        direction LR
        AStart[START: 1 point] --> AData3[DATA: 3 points] --> AData2[DATA: 2 points]
    end
    subgraph LapB [Complete lap B: 5 points]
        direction LR
        BStart[START: 1 point] --> BData4[DATA: 4 points]
    end
    Partial --> AStart
    AData2 -->|Next START closes lap A| BStart
    BData4 -->|Next START closes lap B| Next[Start of an incomplete lap]
```

`START` means CT bit 0 is set; its LSN is 1. `DATA` is a normal packet, and
the number is its LSN point count. The first START opens a lap. The next START
closes that lap and belongs to the new one. Points before the first START and
after the last START cannot form a complete lap in this sample, so they do not
enter `points/lap`. In this example, `laps=2`, `range=5..6`, and the integer
average is `points/lap=5`. Fully received packets still enter `points/s`.

Compare `bytes/s` and the RX buffer completion interval with the main loop's
worst-case processing time before deciding whether 6 Hz provides enough room.
Changing the LiDAR's rotation rate does not by itself prove that its bytes per
second change; this probe measures the actual stream. If the ranging rate stays
near 4000 points/s, expect roughly 667 points/lap at 6 Hz and at least 12,000
point-data bytes/s before packet headers. Those are estimates, not measured
results.

The packet counter can be checked on a host without the STM32 toolchain:

```sh
cc -std=c11 -Wall -Wextra -Werror -I. \
  experiments/lidar_rate_probe/scan_meter.c \
  experiments/lidar_rate_probe/scan_meter_test.c \
  -o /tmp/lidar_scan_meter_test
/tmp/lidar_scan_meter_test
```
