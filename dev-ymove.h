#if defined(YMOVE)
  /* YMOVE - PIN DEFS */

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

  //#define SD_SUPPORTED
  #if !defined(SD_SUPPORTED)
    // eMMC
    #define eMMC_DAT0 2
    #define eMMC_DAT1 4
    #define eMMC_DAT2 12
    #define eMMC_DAT3 13
    #define eMMC_CLK 14
    #define eMMC_CMD 15
  #endif

  // RGB LED
  #define RGB_DATA_PIN 48
  #define RGB_POWER_PIN 38

  #define LDO2_PIN 39
  #define PWR_SHUTDOWN 47

  // How many leds in your strip?
  #define RGB_NUM_LEDS 5

  // Define the array of leds
  //CRGB leds[NUM_LEDS];

  #define BRIGHTNESS 100

#endif