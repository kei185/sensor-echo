# Rx Frame
|command | code | ack |
|---|---| ---|
| device ready | -        | "DEVICE READY\0"|
|get status| 0xAA 0xA1| -|
|start scan| 0xAA 0xA2| "SRT SCAN ACK\0"|
|end scan  | 0xAA 0xA3| "END SCAN ACK\0"|

# Tx Frame
|Start of Frame (16bit)  | payload Length (16bit) | CRC (8bit)|Type (8bit)|timestamp (32bit) |  payload   | 
|---| ---|---|---|---|---|
|0xAA55|-|-| 0x00  system| -|system message | 
|0xAA55|-|-| 0x01  Lidar |-| point data| 
|0xAA55|-|-| 0x02  IMU | -|kinematic data| 
|0xAA55|-|-| 0x03  Encoder |-| odometry data| 

The header is 10 bytes. The start-of-frame bytes are `0xAA 0x55`; payload length
and timestamp are big-endian. The payload begins at `tx_buf + 10`, so it can
be written before the header. Payload length counts payload bytes only.

### System Message

The payload is an ASCII line ending in `\r\n`, with no terminating NUL byte.
The frame type is `0x00`. Status replies report the raw status code and the
decoded state of all six monitored modules. `OK` means the entire status byte
is zero; `FAULT` means at least one status bit is set, including a reserved bit.

```text
[SENSOR-ECHO] LiDAR STATUS: OK | code=0x00 | sensor=OK encoder=OK wireless=OK feedback=OK laser=OK data=OK
[SENSOR-ECHO] LiDAR STATUS: FAULT | code=0x09 | sensor=FAULT encoder=OK wireless=OK feedback=FAULT laser=OK data=OK
```

Device information reports the model, firmware major and minor versions,
hardware version, and the 16 serial-number bytes as 32 uppercase hexadecimal
digits in sensor wire order.

```text
[SENSOR-ECHO] LiDAR DEVICE IDENTIFIED | model=151 firmware=1.2 hardware=3 serial=00112233445566778899AABBCCDDEEFF
```

The payload length includes the final `\r\n` bytes. Responses with an invalid
descriptor, incomplete content, or an unsupported type produce no PC frame.


### Lidar Frame payload 
size: 8 Byte * point number
```mermaid
%%{init: {'theme': 'dark'}}%%

packet
+16: "16bit unsigned distance"
+16: "16bit unsigned angle in Q6 degrees [0, 360)"
```

The angle value is degrees multiplied by 64; for example, 45.5° is 2912.
Angles rotate clockwise from the LiDAR's zero direction. The encoded range is
0..23039; 360° wraps to zero.


### IMU Frame payload 
TODO

### Encoder Frame payload 
TODO

### CRC
The CRC field uses the 8-bit `crc_8()` function from libcrc. For now, its input
is only the two payload-length bytes at header offsets 2 and 3. Process each
byte most-significant bit first and write the full 8-bit result at offset 4.
The CRC does not currently cover the other header fields or the payload.

| Parameter | Value |
|---|---|
| Width | 8 bits |
| Polynomial | `x^8 + x^5 + x^4 + 1`, represented as `0x31` without the top bit |
| Initial value | `0x00` |
| Input and output reflection | None |
| Final XOR | `0x00` |
| Check value for ASCII `123456789` | `0xA2` |
