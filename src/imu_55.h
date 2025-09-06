#ifndef IMU_55_H
#define IMU_55_H

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>

class IMU55 {
public:
  IMU55(uint8_t sda=4, uint8_t scl=5, uint8_t addr=0x28, uint16_t sample_ms=100);
  bool begin();                 // init I2C + BNO + restore calib if present
  void update();                // call each loop (drives fusion)
  void getRPY(float& roll, float& pitch, float& yaw);
  bool isCalibrated();          // true when SYS/G/A/M == 3
  void clearCalNVS();           // manual wipe of saved calibration

private:
  bool loadCalibrationFromNVS();   // CONFIG -> offsets -> ext crystal -> NDOF + kick
  void saveCalibrationToNVS();     // save once when fully calibrated
  void i2c_init(int sda, int scl);

  Adafruit_BNO055 bno_;
  Preferences prefs_;
  uint8_t sda_, scl_, addr_;
  uint16_t sample_ms_;
  bool stored_ = false;
  bool resequenced_ = false;
  uint32_t zeroStart_ = 0;
  sensors_event_t e_;              // last event cache
};

#endif
