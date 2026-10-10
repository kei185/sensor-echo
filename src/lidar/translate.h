#ifndef LIDAR_TRANSLATE_H
#define LIDAR_TRANSLATE_H

#include <stddef.h>
#include <stdint.h>

/* toはCORE_TX_BUF_SIZE byteのTXフレーム先頭。戻り値は全体の長さで、失敗時は0。 */
size_t translate_scan_frame(uint8_t* to);
size_t translate_single(uint8_t* to);

#endif /* LIDAR_TRANSLATE_H */
