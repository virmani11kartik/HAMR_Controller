#ifndef IMU_85_H
#define IMU_85_H

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_BNO08x.h>

class IMU_85 {
public:
    IMU_85(uint8_t sda_pin = 5, uint8_t scl_pin = 4, uint32_t i2c_freq = 100000);
    bool begin();
    bool readOrientation(float& roll, float& pitch, float& yaw, float& accuracy);
    bool dataAvailable();
    bool getQuaternion(float& qr, float& qi, float& qj, float& qk, float& accuracy);
    bool enableRotationVector(uint32_t interval_us = 10000);
    
private:
    // Hardware configuration
    uint8_t _sda_pin;
    uint8_t _scl_pin;
    uint32_t _i2c_freq;
    
    Adafruit_BNO08x _bno08x;
    static const uint8_t BNO08X_ADDR_PRIMARY = 0x4B;
    static const uint8_t BNO08X_ADDR_SECONDARY = 0x4A;
    static const sh2_SensorId_t ORI_REPORT = SH2_ROTATION_VECTOR;
    
    void i2cInit();
    void quatToYPR(float qr, float qi, float qj, float qk, float& yaw, float& pitch, float& roll);
};

#endif // IMU_85_H

/// MAIN CODE ///

// #include <Arduino.h>
// #include "imu_85.h"

// IMU_85 sense;

// static const uint16_t SAMPLE_MS = 100;

// void setup() {
//     Serial0.begin(115200);
//     while (!Serial0) {}
//     if (!sense.begin()) {
//         Serial0.println("Failed to initialize IMU!");
//         while (1) delay(1000);
//     }
//     Serial0.println("IMU ready!");
// }

// void loop() {
//     float roll, pitch, yaw, accuracy;
//     if (sense.readOrientation(roll, pitch, yaw, accuracy)) {
//         Serial0.print("Roll: "); Serial0.print(roll, 2); Serial0.print("° | ");
//         Serial0.print("Pitch: "); Serial0.print(pitch, 2); Serial0.print("° | ");
//         Serial0.print("Yaw: "); Serial0.print(yaw, 2); Serial0.print("° | ");
//         Serial0.print("Acc(rad): "); Serial0.println(accuracy, 3);
//     } else {
//         Serial0.println("No data");
//     }
    
//     delay(SAMPLE_MS);
// }
