#include "tx/scan.h"

#include "lidar/parser.h"
#include "tx/header.h"

_Static_assert(sizeof(ParserScannedPoint) == 4u, "PC scan points must be four bytes");

size_t tx_scan_frame_write(
        const uint8_t* packet,
        size_t         packet_len,
        uint8_t*       tx_buf,
        size_t         tx_capacity,
        uint32_t       timestamp)
{
        if (packet == NULL || tx_buf == NULL || packet_len < SYS_PACKET_SCAN_FIXED_SIZE)
                return 0u;

        const uint8_t lsn = packet[SYS_PACKET_SCAN_HEADER_SIZE + SYS_PACKET_SCAN_CT_SIZE];
        const size_t  payload_len = (size_t)lsn * sizeof(ParserScannedPoint);
        if (lsn == 0u || tx_capacity < TX_FRAME_HEADER_SIZE + payload_len ||
            packet_len <
                    SYS_PACKET_SCAN_FIXED_SIZE + (size_t)lsn * SYS_PACKET_POINT_DATA_SIZE)
                return 0u;

        uint8_t* payload = tx_buf + TX_FRAME_HEADER_SIZE;
        size_t   points  = read_scan_frame(packet, packet_len, payload, payload_len);
        if (points != lsn)
                return 0u;

        return tx_frame_write_header(
                tx_buf,
                tx_capacity,
                (uint16_t)payload_len,
                FRAME_TYPE_LIDAR,
                timestamp);
}
