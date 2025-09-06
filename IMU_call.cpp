#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <utility/imumaths.h>
  
uint16_t BNO055_SAMPLERATE_DELAY_MS = 100;
Adafruit_BNO055 bno = Adafruit_BNO055(55, 0x28, &Wire);

void setup(void)
{
  Serial.begin(115200);
  Wire.begin(4, 5); // SDA=4, SCL=5
  
  while (!Serial) delay(10);

  Serial.println("Orientation Sensor Test"); 
  Serial.println("");

  // int retries = 0;
  // while (!bno.begin() && retries < 5)
  // {
  //   Serial.print("Attempt ");
  //   Serial.print(retries + 1);
  //   Serial.println(": BNO055 not detected, retrying...");
  //   delay(1000);
  //   retries++;
  // }
  
  // if (retries >= 5) {
  //   Serial.println("Failed to initialize BNO055 after 5 attempts!");
  //   Serial.println("Check wiring and power supply");
  //   while (1);
  // }
  
  Serial.println("BNO055 initialized successfully!");
  delay(1000);
}

void printEvent(sensors_event_t* event);

void loop(void)
{
  uint8_t system, gyro, accel, mag = 0;
  bno.getCalibration(&system, &gyro, &accel, &mag);
  
  // Print calibration status with instructions
  Serial.print("Calibration Status - Sys=");
  Serial.print(system);
  Serial.print(" Gyro=");
  Serial.print(gyro);
  Serial.print(" Accel=");
  Serial.print(accel);
  Serial.print(" Mag=");
  Serial.print(mag);
  
  // Provide calibration guidance
  if (system < 3) {
    Serial.print(" | CALIBRATION NEEDED: ");
    if (gyro < 3) Serial.print("Keep STILL for gyro | ");
    if (accel < 3) Serial.print("Place in 6 orientations for accel | ");
    if (mag < 3) Serial.print("Move in figure-8 pattern for mag | ");
  } else {
    Serial.print(" | FULLY CALIBRATED!");
  }
  Serial.println();
  
  // Only show full sensor data if reasonably calibrated
  if (system >= 1) {
    sensors_event_t orientationData , angVelocityData , linearAccelData, magnetometerData, accelerometerData, gravityData;
    bno.getEvent(&orientationData, Adafruit_BNO055::VECTOR_EULER);
    bno.getEvent(&angVelocityData, Adafruit_BNO055::VECTOR_GYROSCOPE);
    bno.getEvent(&linearAccelData, Adafruit_BNO055::VECTOR_LINEARACCEL);
    bno.getEvent(&magnetometerData, Adafruit_BNO055::VECTOR_MAGNETOMETER);
    bno.getEvent(&accelerometerData, Adafruit_BNO055::VECTOR_ACCELEROMETER);
    bno.getEvent(&gravityData, Adafruit_BNO055::VECTOR_GRAVITY);

    printEvent(&orientationData);
    printEvent(&angVelocityData);
    printEvent(&linearAccelData);
    printEvent(&magnetometerData);
    printEvent(&accelerometerData);
    printEvent(&gravityData);

    int8_t boardTemp = bno.getTemp();
    Serial.println();
    Serial.print(F("temperature: "));
    Serial.println(boardTemp);
  }

  Serial.println("--");
  delay(BNO055_SAMPLERATE_DELAY_MS);
}

void printEvent(sensors_event_t* event) {
  double x = -1000000, y = -1000000 , z = -1000000; //dumb values, easy to spot problem
  if (event->type == SENSOR_TYPE_ACCELEROMETER) {
    Serial.print("Accl:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_ORIENTATION) {
    Serial.print("Orient:");
    x = event->orientation.x;
    y = event->orientation.y;
    z = event->orientation.z;
  }
  else if (event->type == SENSOR_TYPE_MAGNETIC_FIELD) {
    Serial.print("Mag:");
    x = event->magnetic.x;
    y = event->magnetic.y;
    z = event->magnetic.z;
  }
  else if (event->type == SENSOR_TYPE_GYROSCOPE) {
    Serial.print("Gyro:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_ROTATION_VECTOR) {
    Serial.print("Rot:");
    x = event->gyro.x;
    y = event->gyro.y;
    z = event->gyro.z;
  }
  else if (event->type == SENSOR_TYPE_LINEAR_ACCELERATION) {
    Serial.print("Linear:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else if (event->type == SENSOR_TYPE_GRAVITY) {
    Serial.print("Gravity:");
    x = event->acceleration.x;
    y = event->acceleration.y;
    z = event->acceleration.z;
  }
  else {
    Serial.print("Unk:");
  }

  Serial.print("\tx= ");
  Serial.print(x);
  Serial.print(" |\ty= ");
  Serial.print(y);
  Serial.print(" |\tz= ");
  Serial.println(z);
}

// #include <Arduino.h>
// #include <Wire.h>
// #include <Preferences.h>
// #include <Adafruit_Sensor.h>
// #include <Adafruit_BNO055.h>
// #include <utility/imumaths.h>

// static const uint8_t SDA_PIN = 4, SCL_PIN = 5;
// static const uint8_t BNO_ADDR = 0x28;      
// static const uint16_t SAMPLE_MS = 100;

// Adafruit_BNO055 bno(55, BNO_ADDR, &Wire);
// Preferences prefs;

// bool loadCalibrationFromNVS() {
//   adafruit_bno055_offsets_t offs;
//   prefs.begin("imu", /*readOnly=*/true);
//   bool has = prefs.isKey("calib");
//   if (has) prefs.getBytes("calib", &offs, sizeof(offs));
//   prefs.end();
//   bno.setMode(OPERATION_MODE_CONFIG);
//   delay(25);
//   if (has) {
//     bno.setSensorOffsets(offs);
//     delay(10);
//     Serial.printf("Restored Offsets | "
//                   "ACC:%d,%d,%d  GYR:%d,%d,%d  MAG:%d,%d,%d  RAD(acc,mag):%d,%d\n",
//       offs.accel_offset_x, offs.accel_offset_y, offs.accel_offset_z,
//       offs.gyro_offset_x,  offs.gyro_offset_y,  offs.gyro_offset_z,
//       offs.mag_offset_x,   offs.mag_offset_y,   offs.mag_offset_z,
//       offs.accel_radius,   offs.mag_radius);
//   }
//   bno.setExtCrystalUse(true);
//   delay(10);
//   bno.setMode(OPERATION_MODE_NDOF);
//   delay(20);
//   sensors_event_t throwaway;
//   for (int i = 0; i < 3; ++i) {
//     bno.getEvent(&throwaway, Adafruit_BNO055::VECTOR_EULER);
//     delay(10);
//   }
//   return has;
// }

// void saveCalibrationToNVS() {
//   adafruit_bno055_offsets_t offs;
//   if (!bno.isFullyCalibrated()) return;
//   bno.getSensorOffsets(offs);
//   prefs.begin("imu", false);
//   prefs.putBytes("calib", &offs, sizeof(offs));
//   prefs.end();
//   Serial.println("Calibration saved to NVS.");
// }

// void clearCalNVS() {
//   prefs.begin("imu", false);  
//   prefs.remove("calib");       
//   prefs.end();
//   Serial.println("Calibration offsets cleared from NVS.");
// }

// static void i2c_init(int sda, int scl) {
//   pinMode(sda, INPUT_PULLUP);
//   pinMode(scl, INPUT_PULLUP);
//   delay(2);
//   if (digitalRead(sda) == LOW) {
//     pinMode(scl, OUTPUT);
//     for (int i = 0; i < 9; ++i) {
//       digitalWrite(scl, LOW);  delayMicroseconds(5);
//       digitalWrite(scl, HIGH); delayMicroseconds(5);
//     }
//     pinMode(sda, OUTPUT);
//     digitalWrite(sda, LOW);    delayMicroseconds(5);
//     digitalWrite(scl, HIGH);   delayMicroseconds(5);
//     digitalWrite(sda, HIGH);   delayMicroseconds(5);
//   }
// }

// void setup() {
//   Serial.begin(115200);
//   while (!Serial) {}
//   Serial.println("Init BNO055…");
//   i2c_init(SDA_PIN, SCL_PIN);

//   Wire.begin(SDA_PIN, SCL_PIN, 100000);
//   Wire.setTimeOut(50);       
//   delay(800);               

//   Serial.printf("Probing 0x%02X… ", BNO_ADDR);
//   Wire.beginTransmission(BNO_ADDR);
//   uint8_t err = Wire.endTransmission(true);
//   Serial.printf("err=%u\n", err);
//   if (err != 0) {
//     Serial.println("No ACK from BNO055 address. Check pins/address (ADR), power, or wiring.");
//   }

//   bool ok = false;
//   for (int tries = 1; tries <= 5; ++tries) {
//     Serial.printf("  Attempt %d: bno.begin()… ", tries);
//     ok = bno.begin();
//     Serial.println(ok ? "OK" : "NOT DETECTED");
//     if (ok) break;
//     delay(500);
//   }
//   if (!ok) {
//     Serial.println("No BNO055 after 5 attempts. Halting.");
//     while (1) delay(1000);
//   }

//   bool restored = loadCalibrationFromNVS();
//   Serial.println(restored ? "Calibration restored from NVS. Move slightly to settle."
//                           : "No saved calibration. Do gyro still / 6 faces / figure-8.");

//   sensor_t s; bno.getSensor(&s);
//   Serial.printf("Sensor ID: %ld\n", (long)s.sensor_id);

//   // clearCalNVS();
//   // Serial.println("Cleared any saved calibration. Re-calibrating from scratch.");
// }


// void loop() {
//   // if (Serial.available()) {
//   //   char c = Serial.read();
//   //   if (c == 'c' || c == 'C') {
//   //     clearCalNVS();
//   //   }
//   // }
//   sensors_event_t e;
//   bno.getEvent(&e, Adafruit_BNO055::VECTOR_EULER);

//   uint8_t sys=0, g=0, a=0, m=0;
//   bno.getCalibration(&sys, &g, &a, &m);

//   static uint32_t zeroStart = 0;
//   static bool resequenced = false;
//   if (!resequenced) {
//     if (sys==0 && g==0 && a==0 && m==0) {
//       if (zeroStart == 0) zeroStart = millis();
//       if (millis() - zeroStart > 2000) {
//         Serial.println("Calib stuck at zeros; re-sequencing NDOF…");
//         bno.setMode(OPERATION_MODE_CONFIG); delay(25);
//         bno.setExtCrystalUse(true);         delay(10);
//         bno.setMode(OPERATION_MODE_NDOF);   delay(25);
//         for (int i=0;i<3;++i){ bno.getEvent(&e, Adafruit_BNO055::VECTOR_EULER); delay(10); }
//         resequenced = true;
//       }
//     } else {
//       zeroStart = 0;
//     }
//   }

//   static bool stored = false;
//   if (!stored && sys==3 && g==3 && a==3 && m==3) {
//     saveCalibrationToNVS();
//     stored = true;
//   }

//   if (sys==3 && g==3 && a==3 && m==3) {
//     const float yaw   = e.orientation.x;
//     const float roll  = e.orientation.y;
//     const float pitch = e.orientation.z;
//     Serial.print("Roll: ");  Serial.print(roll, 2);   Serial.print("°  |  ");
//     Serial.print("Pitch: "); Serial.print(pitch, 2);  Serial.print("°  |  ");
//     Serial.print("Yaw: ");   Serial.print(yaw, 2);    Serial.println("°");
//     delay(SAMPLE_MS);
//   } else {
//     static uint32_t last = 0;
//     if (millis() - last > 1000) {
//       Serial.printf("Calibrate (SYS,G,A,M)=(%u,%u,%u,%u): ", sys,g,a,m);
//       if (g<3) Serial.print("keep still for gyro; ");
//       if (a<3) Serial.print("6 faces for accel; ");
//       if (m<3) Serial.print("slow figure-8 for mag; ");
//       Serial.println();
//       last = millis();
//     }
//     delay(20);
//   }
// }


#include <Arduino.h>
#include "imu_55.h"

IMU55 sens;                 // SDA=4, SCL=5, addr=0x28 by default
float roll,pitch,yaw;

void setup() {
  Serial.begin(115200);
  while (!Serial) {}
  if (!sens.begin()) {
    Serial.println("IMU init failed. Check wiring/address.");
    while (1) delay(1000);
  }
}

void loop() {
  sens.update();
  sens.getRPY(roll,pitch,yaw);
  Serial.println(sens.isCalibrated() ? "Fully Calibrated" : "Not Calibrated");
  Serial.printf("Roll=%.2f Pitch=%.2f Yaw=%.2f\n", roll,pitch,yaw);
  
  delay(100);
}