
/* PIN DEFS */

#define BUTTON_BOOT 0
#define BUTTON_USER 1

// I2C1 - LSM6DSOX - Accelerometer & Gyro
#define I2C1_SDA 8
#define I2C1_SCL 9

#define LSMDSOX_INT1 3
#define LSMDSOX_INT2 10

// I2C1 - LIS3MDL - Magnetometer
#define LIS3MDL_INTM 35
#define LIS3MDL_DRDY 36

// I2C1 - BMP390
#define BMP390_INT 40

// I2C1 - Battery Monitor
#define MAX17048_INT 11

// I2C1 - RTC
#define RTC_INT 5
#define RTC_EVI 6
#define RTC_CLOCKOUT 37

// UART2 - QWIIC
#define UART2_TX 18
#define UART2_RX 7

// I2C2 - QWIIC
#define I2C2_SDA 16
#define I2C2_SCL 17

/*
#define SD_SUPPORTED
#if defined(SD_SUPPORTED)
  // Define SD SPI PINS

#endif
*/

// RGB LED
#define RGB_DATA_PIN 18
#define RGB_NUM_LEDS 1
#define RGB_POWER_PIN 17
#define RGB_BRIGHTNESS 100

#define LDO2_PIN 39
#define PWR_SHUTDOWN 47



