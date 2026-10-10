#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// extern bool imu_arrived;
// extern bool enc_arrived;

void loop(void);
/* handshakeからscan開始までを進める。失敗時は受信を止め、falseを返す。 */
bool run_startup_sequence(void);

#endif
