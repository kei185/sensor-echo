# Startup and Scan Data Flow

Startup uses blocking UART transfers. Scanning uses circular RX DMA and
software-started TX DMA.

## Startup

The startup order and host messages are defined in
[frame.md](frame.md#startup-sequence).

## Scan path

```mermaid
flowchart LR
    lidar["LiDAR byte stream"] --> rx_dma["RX DMA"]
    rx_dma --> rx_ring["4096-byte circular RX ring"]
    rx_ring --> free{"TX slot free?"}
    free -- No --> pause["Keep CPU read position"]
    pause --> rx_ring
    free -- Yes --> overrun{"DMA passed reader?"}
    overrun -- Yes --> drop["Drop partial data<br/>Move reader to DMA position"]
    drop --> fresh["Wait for fresh bytes"]
    overrun -- No --> parse["Parse completed bytes"]
    fresh --> parse
    parse --> frame["Build one host frame"]
    frame --> tx_queue["Five TX slots"]
    tx_queue --> tx_dma["TX DMA"]
    tx_dma --> host["Host"]
```

RX DMA continues while all TX slots are busy. The CPU read position stays
unchanged, so the next free-slot check can detect whether DMA passed it.

One host frame uses one TX slot. A slot follows this cycle:

```mermaid
stateDiagram-v2
    [*] --> Free
    Free --> Filling
    Filling --> Queued
    Queued --> Transmitting
    Transmitting --> Free: UART transmission complete
```

## RX ring positions

`write_idx` is the next DMA destination. `read_idx` is the next byte for the
CPU. The CPU never reads the current DMA destination.

```text
index       0                                             4095
            |-----------------------------------------------|
DMA         W -> next write
CPU                         R -> next read
```

`lap` is the number of DMA wraps that the reader has not crossed yet. DMA
increments it at the ring end. The reader decrements it when its index returns
to zero.

| `lap` | Index relation | Result |
| ---: | --- | --- |
| 0 | Any valid indexes | DMA has not passed the reader |
| 1 | `write_idx <= read_idx` | DMA has not passed the reader |
| 1 | `write_idx > read_idx` | DMA passed the reader |
| 2 or more | Any valid indexes | DMA passed the reader |

```mermaid
flowchart TD
    start["Read lap and indexes"] --> many{"lap > 1?"}
    many -- Yes --> passed["Overrun"]
    many -- No --> one{"lap == 1?"}
    one -- No --> safe["No overrun"]
    one -- Yes --> ahead{"write_idx > read_idx?"}
    ahead -- Yes --> passed
    ahead -- No --> safe
```

The indexes are signed values. This keeps `100 - 3500 = -3400` negative
after one DMA wrap instead of wrapping to a large unsigned value.

## Overrun recovery

```mermaid
flowchart LR
    detect["Detect overrun"] --> discard["Discard unread and partial data"]
    discard --> align["Set read position to current DMA position"]
    align --> wait["Wait for new bytes"]
    wait --> sync["Find the next valid packet"]
    sync --> resume["Resume frame conversion"]
```

Recovery starts from bytes that arrive after alignment. It does not wait for
another full ring wrap.

## Data checks

```mermaid
flowchart LR
    progress["lap and indexes"] --> overwrite["Detect ring overwrite"]
    marker["Packet header"] --> boundary["Find packet boundary"]
    cs["LiDAR packet CS"] --> packet["Check packet bytes"]
    lastcrc["LiDAR LastCRC"] --> rotation["Check a completed rotation"]
    hostcrc["Host frame CRC"] --> hostfield["Check host length field"]
```

The ring position detects overwritten bytes. A checksum cannot replace that
check. LiDAR packet `CS` and `LastCRC` are future checks. The current host
CRC covers only the two payload-length bytes.

## Frame and TX storage sizes

| Item | Largest size |
| --- | ---: |
| One LiDAR packet with one-byte LSN | `10 + 3 * 255 = 775` bytes |
| One host LiDAR frame | `10 + 4 * 255 = 1030` bytes |
| One TX slot | 1344 bytes |

```text
TX storage: 6720 bytes
+-----------------+-----------------+-----------------+-----------------+-----------------+
| slot 0: 1344 B  | slot 1: 1344 B  | slot 2: 1344 B  | slot 3: 1344 B  | slot 4: 1344 B  |
+-----------------+-----------------+-----------------+-----------------+-----------------+
0                1344              2688              4032              5376              6720

Largest frame: [header 10 B][255 points x 4 B][unused 314 B]
```

One slot can be transmitting while four complete frames wait. The payload
arrays use 6720 bytes. The full queue object uses 6764 bytes after metadata
and alignment.

More slots absorb a short burst. They do not increase the sustained UART
rate. With 8N1 framing, the maximum payload rate is approximately
`baud rate / 10` bytes per second.

For host commands and frame fields, see [frame.md](frame.md).
