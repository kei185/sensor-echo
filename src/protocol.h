#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// extern bool imu_arrived;
// extern bool enc_arrived;

void loop(void);

// UART2の4バイト受信完了コールバックから呼び、motor更新と次の受信設定を行う。
void host_command_handler(void);

#endif
