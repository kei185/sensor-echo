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

※ payload size is a multiple of 8 bit  

### System Message
TODO


### Lidar Frame payload 
size: 8 Byte * point number
```mermaid
%%{init: {'theme': 'dark'}}%%

packet
+16: "16bit unsigned distance"
+16: "16bit signed angle (180,-179)"
```

The angle is wrapped into the signed range, then truncated to whole degrees.
0° follows the T-mini Plus zero direction in the data sheet's Figure 4, and positive
angles rotate clockwise. Sensor angles from 181° to 359° map to -179° to -1°.


### IMU Frame payload 
TODO

### Encoder Frame payload 
TODO

### CRC
|||
|---|---|
| Generator  |  CRC-8|
| CRC Width|  7bit |
| CRC Byte Field[7] | reserved |
| CRC Byte Field[6:0] | CRC |
