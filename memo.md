

右 1 A
左 2 B
Enoderの端子を下向きにして時計回り

M2 -> + -> xOUT1  
M1 -> - -> xOUT2  
C1 -> CH2  
C2 -> CH1  
MODE=1  
PHASE=0  

### Encoder Counts

- Each A/B channel gives 7 signal cycles per motor shaft turn.
- The timer counts both edges of both channels in TI12 mode. This gives 4 counts per cycle.
- The gear ratio is 1:100. The motor shaft turns 100 times per wheel turn.

The count per wheel turn is:

```text
7 cycles x 4 counts x 100 turns = 2800 counts
```

The factor of 4 already includes both channels. Do not multiply by 2 again.
### Motor Speed Measurements

On the speed measurement branch, both motors ran at the same duty for 3001 ms.
The travel distance was measured by hand.

| PWM duty | Left turns | Left rpm | Right turns | Right rpm | Distance | Average speed |
| --- | --- | --- | --- | --- | --- | --- |
| 25% | 1.58 rev | 31.50 | 1.55 rev | 31.00 | 0.215 m | 0.0716 m/s |
| 50% | 3.66 rev | 73.25 | 3.61 rev | 72.23 | 0.51 m | 0.1699 m/s |
| 75% | 5.61 rev | 112.16 | 5.64 rev | 112.77 | 0.75 m | 0.2499 m/s |
| 100% | 7.72 rev | 154.34 | 7.58 rev | 151.59 | 1.10 m | 0.3665 m/s |


## SPI Wiring

MOSI -> SDA  
MISO -> SA0  


Lidar 青  
IMU 緑  
PROTOCOL 白  
MOTOR 赤  

