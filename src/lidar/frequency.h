#ifndef LIDAR_FREQUENCY_H
#define LIDAR_FREQUENCY_H

#include <stdbool.h>
#include <stdint.h>

/**
 * スキャン停止中に、UARTのブロッキング通信で設定周波数を調整する。
 * hzは6〜12 Hzの整数値。応答の設定値が目標に一致したらtrueを返す。
 */
bool lidar_set_scan_frequency(uint8_t hz);

#endif /* LIDAR_FREQUENCY_H */
