# Host TX Lifecycle

One host frame uses one TX slot from construction until transmission completes.

## Roles

| Role | Responsibility |
|---|---|
| Frame producer | Builds one complete frame in an empty slot |
| TX queue | Keeps ready frames in production order |
| Transfer arbiter | Starts a transfer when the peripheral is available |
| Completion handler | Releases the transmitted slot |

## Frame lifecycle

```mermaid
flowchart LR
    empty["Empty slot"] --> building["Frame being built"]
    building -- Failed --> empty
    building -- Complete --> ready["Ready frame"]
    ready --> transferring["DMA reading slot"]
    transferring -- Complete --> empty
```

- The producer owns an empty slot while building a frame.
- The queue owns a complete frame until transmission starts.
- DMA owns the selected slot until transmission completes.
- The completion handler returns the slot to the producer.

## Queue heads

| Head | Who advances it | When it advances |
|---|---|---|
| Write head | Frame producer | After publishing one complete frame |
| Read head | Completion handler | After DMA finishes reading one frame |

The queue below has two ready frames. The read head points to the oldest ready
frame. The write head points to the next empty slot.

```text
            read head
                |
                v
+---------+---------+---------+---------+---------+
| slot 0  | slot 1  | slot 2  | slot 3  | slot 4  |
| EMPTY   | READY   | READY   | EMPTY   | EMPTY   |
+---------+---------+---------+---------+---------+
                                  ^
                                  |
                              write head
```

The producer builds a frame in slot 3. Publishing the completed frame advances
the write head to slot 4. The read head does not move.

```text
            read head
                |
                v
+---------+---------+---------+---------+---------+
| slot 0  | slot 1  | slot 2  | slot 3  | slot 4  |
| EMPTY   | READY   | READY   | READY   | EMPTY   |
+---------+---------+---------+---------+---------+
                                            ^
                                            |
                                        write head
```

The arbiter starts DMA from slot 1. Starting a transfer does not move either
head. The read head protects the slot while DMA reads it.

```text
            read head
                |
                v
+---------+---------+---------+---------+---------+
| slot 0  | slot 1  | slot 2  | slot 3  | slot 4  |
| EMPTY   | DMA TX  | READY   | READY   | EMPTY   |
+---------+---------+---------+---------+---------+
                                            ^
                                            |
                                        write head
```

DMA completion releases slot 1 and advances the read head to slot 2.

```text
                      read head
                          |
                          v
+---------+---------+---------+---------+---------+
| slot 0  | slot 1  | slot 2  | slot 3  | slot 4  |
| EMPTY   | EMPTY   | READY   | READY   | EMPTY   |
+---------+---------+---------+---------+---------+
                                            ^
                                            |
                                        write head
```

## Queue rules

- Only a complete, nonempty frame that fits in one slot becomes ready.
- A failed frame build leaves the current write slot empty.
- A transmitting slot cannot be reused.
- A slot becomes empty only after DMA completion.
- If no slot is empty, the producer waits without consuming more LiDAR bytes.

RX DMA continues while the TX queue is full. The RX overrun check detects
whether unread LiDAR data was overwritten. The RX behavior is described in
[protocol.md](protocol.md).

## Capacity

| Item | Largest size |
|---|---:|
| One LiDAR packet with one-byte LSN | `10 + 3 * 255 = 775` bytes |
| One host LiDAR frame | `10 + 4 * 255 = 1030` bytes |
| One TX slot | 1344 bytes |
| Number of TX slots | 5 |

```text
TX storage: 6720 bytes
+-----------------+-----------------+-----------------+-----------------+-----------------+
| slot 0: 1344 B  | slot 1: 1344 B  | slot 2: 1344 B  | slot 3: 1344 B  | slot 4: 1344 B  |
+-----------------+-----------------+-----------------+-----------------+-----------------+
0                1344              2688              4032              5376              6720

Largest frame: [header 10 B][255 points x 4 B][unused 314 B]
```

One slot can be transmitting while four complete frames wait. The payload
arrays use 6720 bytes. The full queue object uses 6764 bytes after metadata and
alignment.

More slots absorb a short burst. They do not increase the sustained UART rate.
With 8N1 framing, the maximum transmitted byte rate is approximately
`baud rate / 10`.
