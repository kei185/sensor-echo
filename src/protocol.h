#ifndef PROTOCOL_H
#define PROTOCOL_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

// extern bool imu_arrived;
// extern bool enc_arrived;

void loop(void);
/* `to` needs CORE_TX_BUF_SIZE bytes. Return the complete host frame size,
 * or zero if translation fails. */
size_t translate_single(uint8_t* to);

#endif
