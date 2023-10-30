/* 
  YMove Device Test Firmware
  By Sukesh Ashok Kumar

  - LS6DSOX + LIS3DL
  - RTC - RV8803 / RV3028-C7
  - MAX17048
  - BMP390
  - RGB LED
*/

#define LED_LSM6DSOX  0
#define LED_LIS3MDL   1
#define LED_BMP390    2
#define LED_RTC       3
#define LED_MAX17048  4

// #define YMOVE
#if defined(YMOVE)
  // YMOVE Device config
  #include "dev-ymove.h"
#else
  // Test device with TinyS3
  #include "dev-tinys3.h"
#endif

// RGB LED
#include <FastLED.h>  // http://librarymanager/All#FastLED

// Simple Fusion
#include <simpleFusion.h>  // http://librarymanager/All#simplefusion
SimpleFusion fuser;               

// LSM6DSOX
#include <Adafruit_LSM6DSOX.h>  // http://librarymanager/All#Adafruit_LSM6DS
Adafruit_LSM6DSOX lsm6ds;

// LIS3MDL
#include <Adafruit_LIS3MDL.h>   // http://librarymanager/All#Adafruit_LIS3MDL
Adafruit_LIS3MDL lis3mdl;

// BMP390
#include <Adafruit_Sensor.h>
#include "Adafruit_BMP3XX.h"  // http://librarymanager/All#Adafruit_BMP3XX
#define SEALEVELPRESSURE_HPA (1013.25)
Adafruit_BMP3XX bmp;

//The below variables control what the date and time will be set to
int sec = 2;
int minute = 47;
int hour = 14;    //Set things in 24 hour mode
int date = 2;
int month = 3;
int year = 2023;
int weekday = 2;  

// YMOVE ONLY
#if defined(YMOVE)
  // RTC - RV8803
  #include <SparkFun_RV8803.h> // http://librarymanager/All#SparkFun_RV-8803
  RV8803 rtc;

  #include "Adafruit_MAX1704X.h" // http://librarymanager/All#Adafruit_MAX1704X
  Adafruit_MAX17048 maxlipo;

#else
  // RTC - RV3028-C7
  #include <RV-3028-C7.h> // http://librarymanager/All#Constantin_Koch_RV-3028-C7
  RV3028 rtc;

#endif

// Define the array of leds
CRGB leds[RGB_NUM_LEDS];

// RGB LED STATUS
void show_led_status(int ledid, uint32_t r, uint32_t g, uint32_t b);

// BMP
void setup_bmp(void);
void show_bmp(void);

// RTC
void setup_rtc(void);
void show_rtc(void);

// IMU
void setup_imu(void);
void show_imu(void);


// ********** SETUP
void setup(void) {
  // ********** Serial init
  Serial.begin(115200);
  
  while (!Serial); // will pause until serial console opens

  Serial.println("*** YMOVE TEST FIRWARE ***");

  // ********** RGB Init and enable RGB LDO 
  Serial.println("[RGB LED Test]");
  FastLED.addLeds<NEOPIXEL, RGB_DATA_PIN>(leds, RGB_NUM_LEDS);  // GRB ordering is assumed
  pinMode(RGB_POWER_PIN, OUTPUT);
  digitalWrite(RGB_POWER_PIN, HIGH);
  FastLED.setBrightness(RGB_BRIGHTNESS);
  blink_rgb();
  
  // ********** RTC Set/Get DateTime
  Serial.println("[RTC Test]");  
  setup_rtc();
  show_rtc();

  // ********** BMP390 Set/Get
  Serial.println("[BP390 Test]");  
  setup_bmp();
  show_bmp();

// Battery Juice
#if defined(YMOVE)
  Serial.println("[MAX17048 Test]");
  if (!maxlipo.begin()) {
    Serial.println(F("Couldnt find MAX17048?\nMake sure a battery is plugged in!"));
  }
  else {
    Serial.print(F("Found MAX17048"));
    Serial.print(F(" with Chip ID: 0x")); 
    Serial.println(maxlipo.getChipID(), HEX);

    Serial.print("Batt Voltage: "); Serial.print(maxlipo.cellVoltage(), 3); Serial.println(" V");
    Serial.print("Batt Percent: "); Serial.print(maxlipo.cellPercent(), 1); Serial.println(" %");
    Serial.println();
  }
#else

#endif

// ********** IMU Init
random16_set_seed(8934);
random16_add_entropy(analogRead(3));
Serial.println("[IMU Test]");  
setup_imu();


// Long delay before Fusion random colors for IMU
delay(1000);
}

// ********** LOOP
void loop() {
  show_imu();
}


void show_led_status(int ledid, uint32_t r, uint32_t g, uint32_t b) {
  if (ledid == -1) {  // IMU random colors
    int i=0;
    for(i=0;i<RGB_NUM_LEDS;i++) {
      leds[i] = CRGB(r,g,b);
    }
    FastLED.show();
    delay(10);
  } else {  // Specific led

#if defined(YMOVE)
    leds[ledid] = CRGB(r,g,b);
#else 
    leds[0] = CRGB(r,g,b);
#endif

    FastLED.show();
    delay(10);
  }
}


// ********** RGB Blink
void blink_rgb(void) {
  int i=0;
  // Show RED
  for(i=0;i<RGB_NUM_LEDS;i++) {
    leds[i] = CRGB::Red;
  }
  FastLED.show();
  delay(500);

  // Show GREEN
  for(i=0;i<RGB_NUM_LEDS;i++) {
    leds[i] = CRGB::Green;
  }
  FastLED.show();
  delay(500);

  // Show BLUE
  for(i=0;i<RGB_NUM_LEDS;i++) {
    leds[i] = CRGB::Blue;
  }
  FastLED.show();
  delay(500);

  // Show BLACK/OFF
  for(i=0;i<RGB_NUM_LEDS;i++) {
    leds[i] = CRGB::Black;
  }
  FastLED.show();
  delay(500);  

  Serial.println();
}

// ********** BMP390
void setup_bmp() {
  if (!bmp.begin_I2C()) {   // hardware I2C mode, can pass in address & alt Wire
    Serial.println("Could not find a valid BMP3 sensor, check wiring!");
    show_led_status(LED_BMP390,255,0,0);	// RED
  }
  else {
    // Set up oversampling and filter initialization
    bmp.setTemperatureOversampling(BMP3_OVERSAMPLING_8X);
    bmp.setPressureOversampling(BMP3_OVERSAMPLING_4X);
    bmp.setIIRFilterCoeff(BMP3_IIR_FILTER_COEFF_3);
    bmp.setOutputDataRate(BMP3_ODR_50_HZ);    
    show_led_status(LED_BMP390,0,255,0);	// GREEN
  }
}

void show_bmp() {
  if (!bmp.performReading()) {
    Serial.println("Failed to perform reading :(");
  }
  else {
  Serial.print("Temperature = ");
  Serial.print(bmp.temperature);
  Serial.println(" *C");

  Serial.print("Pressure = ");
  Serial.print(bmp.pressure / 100.0);
  Serial.println(" hPa");

  Serial.print("Approx. Altitude = ");
  Serial.print(bmp.readAltitude(SEALEVELPRESSURE_HPA));
  Serial.println(" m");

  Serial.println();    
  }
}

// ********** RTC
void setup_rtc(void) {
  Wire.begin();
  Serial.println("Set/Get Time on RTC");

  if (rtc.begin() == false)
  {
    show_led_status(3,255,0,0);
    Serial.println("Device not found. Please check wiring. Freezing.");
    while(1);
  }

  Serial.println("RTC online! Setting Date/Time");
  if (rtc.setTime(sec, minute, hour, weekday, date, month, year) == false) {
    Serial.println("Something went wrong setting the time");
    }
  rtc.set24Hour(); //Uncomment this line if you'd like to set the RTC to 24 hour mode
  show_led_status(3,0,255,0);
}

void show_rtc(void) {
  if (rtc.updateTime() == true) //Updates the time variables from RTC
  {
    //String currentDate = rtc.stringDateUSA(); //Get the current date in mm/dd/yyyy format (we're weird)
    String currentDate = rtc.stringDate();      //Get the current date in dd/mm/yyyy format
    String currentTime = rtc.stringTime();      //Get the time
    Serial.print("Date/Time from RTC: ");
    Serial.print(currentDate);
    Serial.print(" ");
    Serial.println(currentTime);
    Serial.println();
  }
  else
  {
    Serial.print("RTC read failed");
  }    
}

void setup_imu(void) {
  fuser.init(1000000, 0.98, 0.98);    // Initialize the fusion object with the filter update rate (hertz) and 
  bool lsm6ds_success, lis3mdl_success;

  lsm6ds_success = lsm6ds.begin_I2C();
  lis3mdl_success = lis3mdl.begin_I2C();

  if (!lsm6ds_success){
    Serial.println("Failed to find LSM6DSOX chip");
  }
  if (!lis3mdl_success){
    Serial.println("Failed to find LIS3MDL chip");
  }
  if (!(lsm6ds_success && lis3mdl_success)) {
    while (1) {
      delay(10);
    }
  }

  Serial.println("LSM6DSOX and LIS3MDL Found!");

  // lsm6ds.setAccelRange(LSM6DS_ACCEL_RANGE_2_G);
  Serial.print("Accelerometer range set to: ");
  switch (lsm6ds.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  // lsm6ds.setAccelDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (lsm6ds.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  Serial.println();

  // lsm6ds.setGyroRange(LSM6DS_GYRO_RANGE_250_DPS );
  Serial.print("Gyro range set to: ");
  switch (lsm6ds.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }
  // lsm6ds.setGyroDataRate(LSM6DS_RATE_12_5_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (lsm6ds.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }
  Serial.println();

// lsm6ds.configInt1(bool drdy_temp, bool drdy_g, bool drdy_xl);
// lsm6ds.configInt2(bool drdy_temp, bool drdy_g, bool drdy_xl);
// lsm6ds.configIntOutputs(bool active_low, bool open_drain);

// *** Pedometer
// lsm6ds.enablePedometer(true);
// lsm6ds.resetPedometer();
// uint16_t steps = lsm6ds.readPedometer();

// *** enable shake detection
// lsm6ds.enableWakeup(true);

// *** check for shake
// if (lsm6ds.shake()) {
//   Serial.println("SHAKE!");
// }

  lis3mdl.setDataRate(LIS3MDL_DATARATE_155_HZ);
  // You can check the datarate by looking at the frequency of the DRDY pin
  Serial.print("Magnetometer data rate set to: ");
  switch (lis3mdl.getDataRate()) {
    case LIS3MDL_DATARATE_0_625_HZ: Serial.println("0.625 Hz"); break;
    case LIS3MDL_DATARATE_1_25_HZ: Serial.println("1.25 Hz"); break;
    case LIS3MDL_DATARATE_2_5_HZ: Serial.println("2.5 Hz"); break;
    case LIS3MDL_DATARATE_5_HZ: Serial.println("5 Hz"); break;
    case LIS3MDL_DATARATE_10_HZ: Serial.println("10 Hz"); break;
    case LIS3MDL_DATARATE_20_HZ: Serial.println("20 Hz"); break;
    case LIS3MDL_DATARATE_40_HZ: Serial.println("40 Hz"); break;
    case LIS3MDL_DATARATE_80_HZ: Serial.println("80 Hz"); break;
    case LIS3MDL_DATARATE_155_HZ: Serial.println("155 Hz"); break;
    case LIS3MDL_DATARATE_300_HZ: Serial.println("300 Hz"); break;
    case LIS3MDL_DATARATE_560_HZ: Serial.println("560 Hz"); break;
    case LIS3MDL_DATARATE_1000_HZ: Serial.println("1000 Hz"); break;
  }

  lis3mdl.setRange(LIS3MDL_RANGE_4_GAUSS);
  Serial.print("Magnetometer range set to: ");
  switch (lis3mdl.getRange()) {
    case LIS3MDL_RANGE_4_GAUSS: Serial.println("+-4 gauss"); break;
    case LIS3MDL_RANGE_8_GAUSS: Serial.println("+-8 gauss"); break;
    case LIS3MDL_RANGE_12_GAUSS: Serial.println("+-12 gauss"); break;
    case LIS3MDL_RANGE_16_GAUSS: Serial.println("+-16 gauss"); break;
  }

  lis3mdl.setPerformanceMode(LIS3MDL_MEDIUMMODE);
  Serial.print("Magnetometer performance mode set to: ");
  switch (lis3mdl.getPerformanceMode()) {
    case LIS3MDL_LOWPOWERMODE: Serial.println("Low"); break;
    case LIS3MDL_MEDIUMMODE: Serial.println("Medium"); break;
    case LIS3MDL_HIGHMODE: Serial.println("High"); break;
    case LIS3MDL_ULTRAHIGHMODE: Serial.println("Ultra-High"); break;
  }

  lis3mdl.setOperationMode(LIS3MDL_CONTINUOUSMODE);
  Serial.print("Magnetometer operation mode set to: ");
  // Single shot mode will complete conversion and go into power down
  switch (lis3mdl.getOperationMode()) {
    case LIS3MDL_CONTINUOUSMODE: Serial.println("Continuous"); break;
    case LIS3MDL_SINGLEMODE: Serial.println("Single mode"); break;
    case LIS3MDL_POWERDOWNMODE: Serial.println("Power-down"); break;
  }
  Serial.println();

  lis3mdl.setIntThreshold(500);
  lis3mdl.configInterrupt(false, false, true, // enable z axis
                          true, // polarity
                          false, // don't latch
                          true); // enabled!
}

/* 
  Show IMU Data after fusion
*/
void show_imu(void) {
  sensors_event_t accel, gyro, mag, temp;

  //  /* Get new normalized sensor events */
  lsm6ds.getEvent(&accel, &gyro, &temp);
  lis3mdl.getEvent(&mag);

    FusedAngles fusedAngles;                  // Variable to store the output
    /* Fusion settings */
    ThreeAxis accelerometer;                  // Verify that the units are in meters / second ^ 2
    accelerometer.x = accel.acceleration.x;
    accelerometer.y = accel.acceleration.y;
    accelerometer.z = accel.acceleration.z;

    ThreeAxis gyroscope;                      // Verify that the units are in raidans / second
    gyroscope.x = gyro.gyro.x;
    gyroscope.y = gyro.gyro.y;
    gyroscope.z = gyro.gyro.z;
    fuser.getFilteredAngles(accelerometer, gyroscope, &fusedAngles, UNIT_DEGREES);

    Serial.print("Pitch:");
    Serial.print(fusedAngles.pitch);
    Serial.print(",");
    Serial.print("Roll:");
    Serial.println(fusedAngles.roll);

    uint32_t r = (int)abs(fusedAngles.pitch);
    uint32_t g = (int)abs(fusedAngles.roll);
    uint32_t b = (uint32_t)random16();
    show_led_status(-1,r,g,b);
    
}

// ********************* YMOVE ONLY ********************
#if defined(YMOVE)

#else
  // TinyS3

#endif
// ********************* YMOVE ONLY ********************
