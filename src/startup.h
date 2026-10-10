#ifndef STARTUP_H
#define STARTUP_H

#include <stdbool.h>
#include <stdint.h>

/* startup_cancel()の後に呼び、LiDAR情報をhostへ送り、IMUを初期化する。 */
bool startup_initialize_sensors(uint8_t* tx_frame);
/* RX DMAを開始し、LiDARのscan descriptorが届くまで期限付きで待つ。 */
bool startup_start_scan(void);
/* RX DMAとLiDARのscanを止め、RX状態とIMUの受信フラグを初期化する。 */
void startup_cancel(void);

#endif /* STARTUP_H */
