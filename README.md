# YMove-Test
Device Test Firmware 101

### RGB LED STATUS
```c++
// Status LED settings
#define LED_LSM6DSOX  0
#define LED_LIS3MDL   1
#define LED_BMP390    2
#define LED_RTC       3
#define LED_MAX17048  4
```

### EXPECTED SERIAL OUTPUT
```bash
*** YMOVE TEST FIRWARE ***
[RGB LED Test]

[RTC Test]
Set/Get Time on RTC
RTC online! Setting Date/Time
Date/Time from RTC: 14/11/2023 14:47:02

[BP390 Test]
Temperature = 22.46 *C
Pressure = 834.34 hPa
Approx. Altitude = 1609.04 m

[IMU Test]
LSM6DSOX and LIS3MDL Found!
Accelerometer range set to: +-4G
Accelerometer data rate set to: 104 Hz

Gyro range set to: 2000 degrees/s
Gyro data rate set to: 104 Hz

Magnetometer data rate set to: 155 Hz
Magnetometer range set to: +-4 gauss
Magnetometer performance mode set to: Medium
Magnetometer operation mode set to: Continuous

Pitch:0.00,Roll:-0.02
Pitch:0.01,Roll:-0.03
Pitch:0.01,Roll:-0.05
Pitch:0.01,Roll:-0.06
Pitch:0.01,Roll:-0.08
...
```

## DEVICE IO PINS
```
IO01 - BUTTON_USER

IO02 - eMMC_DAT0
IO04 - eMMC_DAT1
IO12 - eMMC_DAT2
IO13 - eMMC_DAT3
IO14 - eMMC_CLK
IO15 - eMMC_CMD

IO03 - LSM6DSOX_INT1
IO10 - LSM6DSOX_INT2
IO35 - LIS3MDL_INTM
IO36 - LIS3MDL_DRDY

IO40 - BMP390 INT

IO05 - RTC_INT
IO06 - RTC_EVI
IO37 - RTC_CLOCKOUT

IO07 - UART2_RX
IO18 - UART2_TX

IO08 - I2C1_SDA
IO09 - I2C1_SCL

IO16 - I2C2_SDA
IO17 - I2C2_SCL

IO11 - MAX17048_INT


IO38 - LED_LDO_Enable
IO48 - LED_DATA

IO39 - LDO2 Enable
IO47 - PWR_SHUTDOWN
```
