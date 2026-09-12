# 起動時の通信シーケンス

以下は起動からスキャン開始後までの設計上の流れ。ステータスと Device Info の取得・PC への転送、PC からの start scan 命令待ちはブロッキング I/O で行う。命令を受けた後に DMA の RX/TX ダブルバッファを設定し、LiDAR にスキャン開始を通知する。

```mermaid
sequenceDiagram
    autonumber
    participant PC
    participant MCU as 制御側
    participant LiDAR as LiDAR（peripheral）
    participant DMA

    Note over PC,LiDAR: 初期化：ブロッキング I/O
    MCU->>LiDAR: ステータス要求
    LiDAR-->>MCU: ステータス受信
    MCU-->>PC: ステータス転送
    MCU->>LiDAR: Device Info 要求
    LiDAR-->>MCU: Device Info 受信
    MCU-->>PC: Device Info 転送
    MCU-->>PC: READY 通知
    PC->>MCU: start scan 命令

    Note over MCU,DMA: ここから DMA を使用
    MCU->>DMA: RX/TX ダブルバッファ設定
    MCU->>LiDAR: スキャン開始通知
    loop スキャン中
        opt RX DMA 完了割り込み
            DMA-->>MCU: RX 完了
            MCU->>DMA: RX バッファ切り替え
        end
        opt TX DMA 完了割り込み
            DMA-->>MCU: TX 完了
            MCU->>DMA: TX バッファ切り替え
        end
    end
```

PC 向けのコマンドと転送フレームの仕様は [frame.md](frame.md) を参照。この図は予定する処理順序を示し、現時点での実装完了を表すものではない。
