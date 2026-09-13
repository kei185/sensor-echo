
#ifndef LIDAR_CORE_H
#define LIDAR_CORE_H

#include <stdint.h>
#include <stdbool.h>

#define CORE_RX_BUF_SIZE 1024u // TODO: バッファサイズを確定する
#define CORE_RX_BUF_NUM  2u    // use DMA as double buffer

typedef struct RxBuf
{
        int8_t  ready_buf_idx; // supposed to be update in DMA IRQ handler
        int8_t* _bufs[CORE_RX_BUF_NUM];
} RxBuf;

extern const RxBuf* const RX_BUF;

int8_t*  cur_buf(void);
bool     is_safe_read(const int8_t*, uint32_t);
int8_t   read_byte(const int8_t*);
uint32_t dec_little_endian(const int8_t*, uint8_t);

#define CORE_TX_BUF_SIZE 1024u // TODO: バッファサイズを確定する
#define CORE_TX_BUF_NUM  2u    // use DMA as double buffer

typedef struct TxBuf
{
        int8_t  ready_buf_idx; // supposed to be update in DMA IRQ handler
        int8_t* _bufs[CORE_RX_BUF_NUM];
} TxBuf;

#endif /* LIDAR_CORE_H */
