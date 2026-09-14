
#ifndef LIDAR_CORE_H
#define LIDAR_CORE_H

#include <stdint.h>
#include <stdbool.h>

#define CORE_RX_BUF_SIZE 4096u // bytes in each RX slot
#define CORE_RX_BUF_NUM  2u

typedef struct RxBuf
{
        int8_t  ready_buf_idx;          // supposed to be update in DMA IRQ handler
        int8_t* _bufs[CORE_RX_BUF_NUM]; // two slices of one contiguous array
} RxBuf;

extern const RxBuf* const RX_BUF;

int8_t*  cur_buf(void);
bool     is_safe_read(const int8_t*, uint32_t);
int8_t   read_byte(const int8_t*);
uint32_t dec_little_endian(const int8_t*, uint8_t);

#define CORE_TX_BUF_SIZE 1344u // bytes in each TX slot; 3 slots = 4032 bytes
#define CORE_TX_BUF_NUM  3u

typedef struct TxBufStat
{
        bool    full;
        bool    transmitting;
        int8_t* _buf; // one slice of the contiguous TX array
} TxBufStat;

typedef struct TxBuf
{
        TxBufStat _bufs[CORE_TX_BUF_NUM];
} TxBuf;

extern const TxBuf* const TX_BUF;

#endif /* LIDAR_CORE_H */
