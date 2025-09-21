#include <Arduino.h>
#include "imu_85.h"

IMU_85 sense;

static const uint16_t SAMPLE_MS = 100;

void setup() {
    Serial0.begin(115200);
    while (!Serial0) {}
    if (!sense.begin()) {
        Serial0.println("Failed to initialize IMU!");
        while (1) delay(1000);
    }
    Serial0.println("IMU ready!");
}

void loop() {
    float roll, pitch, yaw, accuracy;
    if (sense.readOrientation(roll, pitch, yaw, accuracy)) {
        Serial0.print("Roll: "); Serial0.print(roll, 2); Serial0.print("° | ");
        Serial0.print("Pitch: "); Serial0.print(pitch, 2); Serial0.print("° | ");
        Serial0.print("Yaw: "); Serial0.print(yaw, 2); Serial0.print("° | ");
        Serial0.print("Acc(rad): "); Serial0.println(accuracy, 3);
    } else {
        Serial0.println("No data");
    }
    
    delay(SAMPLE_MS);
}