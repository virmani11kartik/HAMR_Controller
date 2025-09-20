// src/main.cpp
#include <Arduino.h>
#include <Wire.h>
#include <ESP32_TWAI.h>   // eyr1n/ESP32_TWAI
#include "imu_85.h"       // your IMU class (unchanged)

// ---------- PINS (override via build_flags in platformio.ini) ----------
#ifndef I2C_SDA_PIN
#define I2C_SDA_PIN 5     // your wiring: SDA=5
#endif
#ifndef I2C_SCL_PIN
#define I2C_SCL_PIN 4     // your wiring: SCL=4
#endif

// Default to safe C3 pins; override with -DTWAI_TX_PIN=18 -DTWAI_RX_PIN=19
// if you're disabling native USB and using a USB-UART for flashing.
#ifndef TWAI_TX_PIN
#define TWAI_TX_PIN 6
#endif
#ifndef TWAI_RX_PIN
#define TWAI_RX_PIN 7
#endif
// ----------------------------------------------------------------------

static const uint16_t CAN_ID_IMU_RPY = 0x123;   // standard 11-bit ID
static const uint16_t PERIOD_MS      = 10;      // 100 Hz

IMU_85 imu(I2C_SDA_PIN, I2C_SCL_PIN, 400000);   // use your class as-is @ 400 kHz

// helper: clamp & scale float -> int16
static inline int16_t f2i16_scaled(float x, float scale) {
  float v = x * scale;
  if (v >  32767.0f) v =  32767.0f;
  if (v < -32768.0f) v = -32768.0f;
  return (int16_t)lroundf(v);
}

void setup() {
  Serial.begin(115200);
  while (!Serial) {}

  // --- IMU init ---
  if (!imu.begin()) {
    Serial.println("IMU init failed. Check wiring/pull-ups/power.");
    while (1) delay(250);
  }
  Serial.println("IMU ready.");

  // --- CAN init (eyr1n/ESP32_TWAI) ---
  // Set pins first (many forks use macros; this call covers the ones that support runtime setup)
  #if defined(CAN)
    CAN.setPins((gpio_num_t)TWAI_TX_PIN, (gpio_num_t)TWAI_RX_PIN);
  #endif
  if (!CAN.begin(CanBitRate::BR_500k)) {     // match your bus (BR_125k/250k/500k/1000k)
    Serial.println("CAN begin failed.");
    while (1) delay(250);
  }
  Serial.printf("CAN started @ 500 kbit/s, TX=%d RX=%d\n", TWAI_TX_PIN, TWAI_RX_PIN);
}

void loop() {
  static uint32_t last = 0;
  const uint32_t now = millis();
  if (now - last < PERIOD_MS) {
    // (Optional) drain RX while we wait
    if (CAN.available()) {
      CanMsg rx = CAN.read();
      // Example: print ID/DLC
      // Serial.printf("RX id=0x%03X dlc=%d\n", rx.getStandardId(), rx.data_length);
    }
    return;
  }
  last = now;

  float roll, pitch, yaw, acc_rad;
  if (imu.readOrientation(roll, pitch, yaw, acc_rad)) {
    // Pack: roll/pitch/yaw (centi-deg), accuracy (millirad) => 8 bytes total
    const int16_t roll_cd   = f2i16_scaled(roll, 100.0f);
    const int16_t pitch_cd  = f2i16_scaled(pitch, 100.0f);
    const int16_t yaw_cd    = f2i16_scaled(yaw, 100.0f);
    const uint16_t acc_mrad = (uint16_t)lroundf(fmaxf(0.0f, fminf(acc_rad * 1000.0f, 65535.0f)));

    CanMsg tx;
    tx.id = CanStandardId(CAN_ID_IMU_RPY);
    tx.data_length = 8;

    memcpy(&tx.data[0], &roll_cd,   2);
    memcpy(&tx.data[2], &pitch_cd,  2);
    memcpy(&tx.data[4], &yaw_cd,    2);
    memcpy(&tx.data[6], &acc_mrad,  2);

    CAN.write(tx);

    // (Optional) debug
    // Serial.printf("RPY(deg): %.2f %.2f %.2f | acc(rad): %.3f\n", roll, pitch, yaw, acc_rad);
  }
}
