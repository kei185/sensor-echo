# Startup and Scan Data Flow

This page describes the planned startup sequence and scan data path. Startup uses blocking I/O. After the PC sends the start scan command, the controller configures DMA double buffers and starts the LiDAR.

RX carries data from the LiDAR to the controller. TX carries data from the controller to the PC.

## Startup sequence

```mermaid
sequenceDiagram
    autonumber
    participant PC
    participant Controller
    participant LiDAR
    participant RxDma as RX DMA
    participant TxDma as TX DMA

    Note over PC,LiDAR: Startup uses blocking I/O
    Controller->>LiDAR: Request status
    LiDAR-->>Controller: Status response
    Controller-->>PC: Forward status
    Controller->>LiDAR: Request device information
    LiDAR-->>Controller: Device information response
    Controller-->>PC: Forward device information
    Controller-->>PC: Device ready notification
    PC->>Controller: Start scan command

    Note over Controller,TxDma: Scan transfer uses DMA
    Controller->>RxDma: Configure two RX buffers
    Controller->>TxDma: Configure two TX buffers
    Controller->>LiDAR: Start scan
    loop While scanning
        par RX DMA completion
            RxDma-->>Controller: RX completion interrupt
            Controller->>RxDma: Switch active RX buffer
            Controller->>Controller: Check TX buffer availability
            alt A TX buffer is free
                Controller->>Controller: Convert RX data into the free TX buffer
                Controller->>Controller: Mark TX buffer ready
                opt TX DMA is idle
                    Controller->>TxDma: Start transfer
                end
            else Both TX buffers are busy
                Controller->>Controller: Discard this RX cycle
            end
        and TX DMA completion
            TxDma-->>Controller: TX completion interrupt
            Controller->>Controller: Release transmitted TX buffer
            Controller->>TxDma: Start queued transfer, if any
        end
    end
```

## Buffer ownership and data path

The software arbiter tracks which RX buffer has completed and whether each TX buffer is free, queued, or transmitting. The two DMA completion interrupts can arrive in either order. Solid arrows show data movement; dashed arrows show completion interrupts.

```mermaid
flowchart LR
    lidar[LiDAR] --> rx_dma[RX DMA]
    subgraph rx_buffers[RX double buffer]
        rx_a[RX buffer A]
        rx_b[RX buffer B]
    end
    rx_dma --> rx_a
    rx_dma --> rx_b
    rx_a -->|Completed data| arbiter{Software arbiter<br/>Free TX buffer?}
    rx_b -->|Completed data| arbiter
    rx_dma -. RX completion interrupt .-> arbiter

    arbiter -->|Yes| convert[Decode scan data<br/>and build PC frame]
    arbiter -->|No| discard[Discard this RX cycle]
    subgraph tx_buffers[TX double buffer]
        tx_a[TX buffer A]
        tx_b[TX buffer B]
    end
    convert -->|Selected free buffer| tx_a
    convert -->|Selected free buffer| tx_b
    tx_a --> tx_dma[TX DMA]
    tx_b --> tx_dma
    tx_dma --> pc[PC]
    tx_dma -. TX completion interrupt .-> arbiter
```

If one TX buffer is transmitting and the other is queued, neither is available. The arbiter discards the completed RX data for that cycle. It does not retry the discarded data; the next completed RX buffer is considered for transfer. After conversion or discard, the completed RX buffer becomes available for reuse.

A TX completion releases the transmitted buffer and starts a queued transfer when one exists.

For PC commands and frame formats, see [frame.md](frame.md). These diagrams describe the intended design, not the current implementation status.
