
#ifndef LIDAR_CORE_H
#define LIDAR_CORE_H

#include <stddef.h>
#include <stdint.h>
#include <stdbool.h>

#define CORE_RX_BUF_SIZE 4096u // byte
typedef struct RxBuf
{

        // suppose that dma is set direct mode, byte wise transfer
        volatile uint32_t* remain_bytes; // updated by DMA
        volatile uint8_t   lap;          // update by DMA RX IRQ
        int8_t*            _buf;
        uint32_t           read_idx;
} RxBuf;

extern const RxBuf* const RX_BUF;

bool     is_lapped(void);
void     reset_read_idx(void);
void     increment_lap(void);
bool     is_safe_read(void);
int8_t   read_byte();
uint32_t dec_little_endian(const uint8_t);
bool     load_blocking_rx(const uint8_t* data, size_t length);
bool     start_lidar_rx_dma(void);

#define CORE_TX_BUF_SIZE 1344u // bytes in each TX slot; 3 slots = 4032 bytes
#define CORE_TX_BUF_NUM  5u

typedef struct
{

        volatile bool full;
        uint32_t      length;
        int8_t        _buf[CORE_TX_BUF_SIZE];
} TxBufSlot;

typedef struct TxBufQueue
{
        uint8_t   writer_head;
        uint8_t   reader_head;
        TxBufSlot slots[CORE_TX_BUF_NUM];
} TxBuf;

void       release(TxBufSlot*);
TxBufSlot* get_full_buf(void);
TxBufSlot* get_empty_buf(void);
void       push_full_slot(TxBufSlot*, uint32_t);

extern const TxBuf* const TX_BUF;

#endif /* LIDAR_CORE_H */
