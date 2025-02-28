#include <Wire.h>
#include <Adafruit_LSM6DSOX.h>
#include <MahonyAHRS.h>
#include <Adafruit_Sensor_Calibration.h>

// Create sensor objects for two IMUs
Adafruit_LSM6DSOX imu1;
Adafruit_LSM6DSOX imu2;

// Create Mahony filter instances for both IMUs
Mahony filter1;
Mahony filter2;

#if defined(ADAFRUIT_SENSOR_CALIBRATION_USE_EEPROM)
  Adafruit_Sensor_Calibration_EEPROM cal;
#else
  Adafruit_Sensor_Calibration_SDFat cal;
#endif

#define FILTER_UPDATE_RATE_HZ 100
#define PRINT_EVERY_N_UPDATES 10
//#define AHRS_DEBUG_OUTPUT

void setup() {
  Serial.begin(115200);
  Wire.begin();

  if (!cal.begin()) {
    Serial.println("Failed to initialize calibration helper");
  } else if (! cal.loadCalibration()) {
    Serial.println("No calibration loaded/found");
  }

  // Initialize IMU 1 (0x6A)
  if (!imu1.begin_I2C(0x6A)) {
    Serial.println("Failed to find IMU 1 at 0x6A");
    while (1);
  } else {
    Serial.println("IMU 1 (0x6A) initialized!");
  }

  // Initialize IMU 2 (0x6B)
  if (!imu2.begin_I2C(0x6B)) {
    Serial.println("Failed to find IMU 2 at 0x6B");
    while (1);
  } else {
    Serial.println("IMU 2 (0x6B) initialized!");
  }

  // Set the filter update rate (100 Hz)
  filter1.begin(100);
  filter2.begin(100);



}

void loop() {
  sensors_event_t accel1, gyro1, temp1;
  sensors_event_t accel2, gyro2, temp2;

  // Read data from IMU 1
  imu1.getEvent(&accel1, &gyro1, &temp1);
  
  // Read data from IMU 2
  imu2.getEvent(&accel2, &gyro2, &temp2);


  cal.calibrate(accel1);
  cal.calibrate(gyro1);
  cal.calibrate(accel2);
  cal.calibrate(gyro2);


  // Convert gyroscope values to radians per second
  //float gyroScale = PI / 180.0f;

  float gx1 = gyro1.gyro.x ;
  float gy1 = gyro1.gyro.y ;
  float gz1 = gyro1.gyro.z ;

  float gx2 = gyro2.gyro.x ;
  float gy2 = gyro2.gyro.y ;
  float gz2 = gyro2.gyro.z ;

  // Update Mahony filter for IMU 1
  filter1.updateIMU(gx1, gy1, gz1, accel1.acceleration.x, accel1.acceleration.y, accel1.acceleration.z);
  float roll1 = filter1.getRoll();
  float pitch1 = filter1.getPitch();
  float yaw1 = filter1.getYaw();

  // Update Mahony filter for IMU 2
  filter2.updateIMU(gx2, gy2, gz2, accel2.acceleration.x, accel2.acceleration.y, accel2.acceleration.z);
  float roll2 = filter2.getRoll();
  float pitch2 = filter2.getPitch();
  float yaw2 = filter2.getYaw();

  // Print the results
  if (readyToPrint()) {
    Serial.print("IMU1 - Yaw: "); Serial.print(yaw1);
    Serial.print(", Pitch: "); Serial.print(pitch1);
    Serial.print(", Roll: "); Serial.println(roll1);

    Serial.print("IMU2 - Yaw: "); Serial.print(yaw2);
    Serial.print(", Pitch: "); Serial.print(pitch2);
    Serial.print(", Roll: "); Serial.println(roll2);
  }
}

// Function to control print frequency
bool readyToPrint() {
  static unsigned long lastPrintTime = 0;
  unsigned long now = millis();

  if (now - lastPrintTime > 125) { // Prints 8 times per second
    lastPrintTime = now;
    return true;
  }
  return false;
}
