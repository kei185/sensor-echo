#include <assert.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "lidar/parser.h"

// A complete scan packet must not wait for a DMA buffer to become readable.
int8_t read_byte(int8_t* buf)
{
        (void)buf;
        abort();
}

uint32_t dec_little_endian(int8_t* buf, uint8_t len)
{
        (void)buf;
        (void)len;
        abort();
}

int main(void)
{
        // Manual sections 3.1.4-3.1.5: 64 E5 6F is 7161 mm. The three FSA/LSA
        // samples span 45 to 55 degrees; CS is present but unchecked.
        const uint8_t      scan[]    = {0xAA,
                                        0x55,
                                        0x00,
                                        0x03,
                                        0x81,
                                        0x16,
                                        0x81,
                                        0x1B,
                                        0x00,
                                        0x00,
                                        0x64,
                                        0xE5,
                                        0x6F,
                                        0x42,
                                        0x00,
                                        0x08,
                                        0x11,
                                        0xFC,
                                        0xFF};
        ParserScannedPoint points[3] = {{0}};
        assert(sizeof(scan) ==
               SYS_PACKET_SCAN_FIXED_SIZE + 3u * SYS_PACKET_POINT_DATA_SIZE);
        assert(read_scan_frame(scan, sizeof(scan), points, 3u) == 3u);
        assert(points[0].angle == 45 && points[0].dist == 7161);
        assert(points[1].angle == 50 && points[1].dist == 512);
        assert(points[2].angle == 55 && points[2].dist == 16383);
        assert(PARSER_SCAN_META->data_num == 3u);
        assert(PARSER_SCAN_META->start_angle_q6 == 45u * 64u);
        assert(PARSER_SCAN_META->end_angle_q6 == 55u * 64u);
        assert(PARSER_SCAN_META->data_frame_head == scan + SYS_PACKET_SCAN_FIXED_SIZE);

        // Clockwise interpolation crosses 360 degrees and normalizes to [-179, 180].
        const uint8_t wrap[] = {0xAA,
                                0x55,
                                0x00,
                                0x03,
                                0x81,
                                0xB3,
                                0x81,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00,
                                0x00};
        assert(read_scan_frame(wrap, sizeof(wrap), points, 3u) == 3u);
        assert(points[0].angle == -1 && points[1].angle == 0 && points[2].angle == 1);

        // Keep the fractional Q6 angle until after interpolation.
        uint8_t fractional[sizeof(scan)];
        memcpy(fractional, scan, sizeof(scan));
        fractional[4] = 0xC1; // 45.5 degrees
        fractional[6] = 0x41; // 46.5 degrees
        fractional[7] = 0x17;
        assert(read_scan_frame(fractional, sizeof(fractional), points, 3u) == 3u);
        assert(points[0].angle == 45 && points[1].angle == 46 && points[2].angle == 46);

        // A start packet has one sample. Degree 180 remains positive in the PC format.
        const uint8_t start[] = {0xAA,
                                 0x55,
                                 0x01,
                                 0x01,
                                 0x01,
                                 0x5A,
                                 0x01,
                                 0x5A,
                                 0x00,
                                 0x00,
                                 0x00,
                                 0x00,
                                 0x00};
        assert(read_scan_frame(start, sizeof(start), points, 3u) == 1u);
        assert(points[0].angle == 180 && points[0].dist == 0);

        // Crossing the signed-angle boundary must happen before truncating Q6.
        uint8_t boundary[sizeof(start)];
        memcpy(boundary, start, sizeof(start));
        boundary[4] = boundary[6] = 0x41; // 180.5 degrees
        assert(read_scan_frame(boundary, sizeof(boundary), points, 3u) == 1u);
        assert(points[0].angle == -179);
        boundary[4] = boundary[6] = 0xC1; // 359.5 degrees
        boundary[5] = boundary[7] = 0xB3;
        assert(read_scan_frame(boundary, sizeof(boundary), points, 3u) == 1u);
        assert(points[0].angle == 0);

        ParserScannedPoint unchanged[3];
        memcpy(unchanged, points, sizeof(points));
        assert(!read_scan_frame(NULL, sizeof(scan), points, 3u));
        assert(!read_scan_frame(scan, sizeof(scan), NULL, 3u));
        assert(!read_scan_frame(scan, sizeof(scan), points, 2u));
        for (uint32_t n = 0; n < sizeof(scan); ++n)
                assert(!read_scan_frame(scan, n, points, 3u));

        uint8_t malformed[sizeof(scan)];
        memcpy(malformed, scan, sizeof(scan));
        malformed[0] = 0x55; // wrong header order
        assert(!read_scan_frame(malformed, sizeof(malformed), points, 3u));
        memcpy(malformed, scan, sizeof(scan));
        malformed[3] = 0; // no samples
        assert(!read_scan_frame(malformed, sizeof(malformed), points, 3u));
        memcpy(malformed, scan, sizeof(scan));
        malformed[4] &= 0xFE; // FSA check bit must be one
        assert(!read_scan_frame(malformed, sizeof(malformed), points, 3u));
        memcpy(malformed, scan, sizeof(scan));
        malformed[6] &= 0xFE; // LSA check bit must be one
        assert(!read_scan_frame(malformed, sizeof(malformed), points, 3u));
        memcpy(malformed, scan, sizeof(scan));
        malformed[2] = 1; // start packet cannot carry three samples
        assert(!read_scan_frame(malformed, sizeof(malformed), points, 3u));
        assert(memcmp(points, unchanged, sizeof(points)) == 0);

        puts("PASS: scan distance, angle interpolation, wrap, start packet, and bounds");
        return 0;
}
