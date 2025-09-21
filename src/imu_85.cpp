#include "imu_85.h"

IMU_85::IMU_85(uint8_t sda_pin, uint8_t scl_pin, uint32_t i2c_freq) 
    : _sda_pin(sda_pin), _scl_pin(scl_pin), _i2c_freq(i2c_freq) {
}

bool IMU_85::begin() {
    Serial0.println("Initializing BNO085 IMU...");
    
    i2cInit();
    Wire.begin(_sda_pin, _scl_pin, _i2c_freq);
    Wire.setTimeOut(50);
    delay(100);
    
    bool ok = _bno08x.begin_I2C(BNO08X_ADDR_PRIMARY, &Wire);
    if (!ok) {
        Serial0.println("Primary addr 0x4B not found; trying 0x4A...");
        ok = _bno08x.begin_I2C(BNO08X_ADDR_SECONDARY, &Wire);
    }
    if (!ok) {
        Serial0.println("No BNO08x detected on I2C (0x4B/0x4A). Check wiring/power.");
        return false;
    }
    Serial0.println("BNO08x found.");
    
    if (!enableRotationVector()) {
        Serial0.println("Failed to enable rotation vector report!");
        return false;
    }
    Serial0.println("BNO085 IMU initialized successfully.");
    return true;
}

bool IMU_85::enableRotationVector(uint32_t interval_us) {
    if (!_bno08x.enableReport(ORI_REPORT, interval_us)) {
        return false;
    }
    Serial0.println("Rotation Vector report enabled.");
    return true;
}

bool IMU_85::dataAvailable() {
    sh2_SensorValue_t val;
    return _bno08x.getSensorEvent(&val);
}

bool IMU_85::getQuaternion(float& qr, float& qi, float& qj, float& qk, float& accuracy) {
    sh2_SensorValue_t val;
    
    while (_bno08x.getSensorEvent(&val)) {
        if (val.sensorId == ORI_REPORT) {
            qi = val.un.rotationVector.i;
            qj = val.un.rotationVector.j;
            qk = val.un.rotationVector.k;
            qr = val.un.rotationVector.real;
            accuracy = val.un.rotationVector.accuracy;
            return true;
        }
    }
    return false;
}

bool IMU_85::readOrientation(float& roll, float& pitch, float& yaw, float& accuracy) {
    float qr, qi, qj, qk;
    
    if (getQuaternion(qr, qi, qj, qk, accuracy)) {
        quatToYPR(qr, qi, qj, qk, yaw, pitch, roll);
        return true;
    }
    return false;
}

void IMU_85::quatToYPR(float qr, float qi, float qj, float qk,
                       float& yaw, float& pitch, float& roll) {
    // yaw (Z), pitch (Y), roll (X)
    float ysqr = qj * qj;
    
    // roll (x-axis rotation)
    float t0 = +2.0f * (qr * qi + qj * qk);
    float t1 = +1.0f - 2.0f * (qi * qi + ysqr);
    roll = atan2f(t0, t1);
    
    // pitch (y-axis rotation)
    float t2 = +2.0f * (qr * qj - qk * qi);
    t2 = t2 > 1.0f ? 1.0f : t2;
    t2 = t2 < -1.0f ? -1.0f : t2;
    pitch = asinf(t2);
    
    // yaw (z-axis rotation)
    float t3 = +2.0f * (qr * qk + qi * qj);
    float t4 = +1.0f - 2.0f * (ysqr + qk * qk);
    yaw = atan2f(t3, t4);
    
    // radians -> degrees
    const float RAD2DEG = 57.2957795f;
    yaw *= RAD2DEG;
    pitch *= RAD2DEG;
    roll *= RAD2DEG;
}

void IMU_85::i2cInit() {
    pinMode(_sda_pin, INPUT_PULLUP);
    pinMode(_scl_pin, INPUT_PULLUP);
    delay(2);
    
    if (digitalRead(_sda_pin) == LOW) {
        pinMode(_scl_pin, OUTPUT);
        for (int i = 0; i < 9; ++i) {
            digitalWrite(_scl_pin, LOW); 
            delayMicroseconds(5);
            digitalWrite(_scl_pin, HIGH); 
            delayMicroseconds(5);
        }
        // STOP
        pinMode(_sda_pin, OUTPUT);
        digitalWrite(_sda_pin, LOW); 
        delayMicroseconds(5);
        digitalWrite(_scl_pin, HIGH); 
        delayMicroseconds(5);
        digitalWrite(_sda_pin, HIGH); 
        delayMicroseconds(5);
    }
}