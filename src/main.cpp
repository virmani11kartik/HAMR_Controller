#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "imu_85.h"
#include "driver/twai.h"

#include <WiFi.h>
#include "esp_wifi.h"

// ---------------------- LED (M5Stamp C3 onboard) ----------------------
#define RGB_PIN 2
Adafruit_NeoPixel led(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- Wi-Fi AP settings -----------------------------
static const char* AP_SSID = "HAMR-SSH";
static const char* AP_PASS = "123571113";
IPAddress ap_ip(192, 168, 50, 1);
IPAddress ap_gw(192, 168, 50, 1);
IPAddress ap_netmask(255, 255, 255, 0);

// ---------------------- IMU (I2C) -------------------------------------
IMU_85 sense(1, 0); // SDA=GPIO1, SCL=GPIO0

// ---------------------- CAN (TWAI) pins & rate ------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_19;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_18;
static const int CAN_HZ = 1000000; // 1 Mbit/s

// ---------------------- Motor protocol --------------------------------
static const uint8_t  MOTOR_ROLL  = 1;
static const uint8_t  MOTOR_PITCH = 2;
static const uint32_t TX_BASE     = 0x140;
static const uint32_t RX_BASE     = 0x240;

static const uint8_t CMD_ABS_POSITION   = 0xA4;
static const uint8_t CMD_READ_STATUS2   = 0x9C;
static const uint8_t CMD_MOTOR_STOP     = 0x81;

// Max speed used for all gimbal position commands (deg/s).
// Raise once you've confirmed direction is correct.
static const uint16_t GIMBAL_MAX_DPS = 100;

// Hard position clamp — motors never commanded beyond this,
// regardless of IMU reading (prevents self-collision).
static const float GIMBAL_MAX_DEG = 20.0f;

// Mechanical mounting offset — motor's encoder zero is this many degrees
// away from the physical neutral position. +90 = motor zero is 90° ACW from level.
static const float ROLL_MOUNT_OFFSET_DEG  = -90.0f;
static const float PITCH_MOUNT_OFFSET_DEG = -90.0f;

// ---------------------- App timing ------------------------------------
static const uint16_t SAMPLE_MS = 20; // 50 Hz

// ---------------------- PD gains --------------------------------------
// Direct edit marker: PD controller enabled for IMU leveling.
static float k_p = 1.0f;
static float k_d = 0.5f;

// ---------------------- State flags -----------------------------------
bool can_ok = false;
bool imu_ok = false;
bool ap_ok  = false;

// ---------------------- TWAI helpers ----------------------------------
static bool twai_send(uint32_t id, const uint8_t data[8]) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.flags            = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

// Drain one reply frame, matching wantID. Ignores frames for other IDs.
static bool twai_recv(uint32_t wantID, twai_message_t& out, uint32_t timeout_ms = 20) {
    uint32_t deadline = millis() + timeout_ms;
    while (millis() < deadline) {
        uint32_t rem = deadline - millis();
        twai_message_t m;
        if (twai_receive(&m, pdMS_TO_TICKS(rem > 0 ? rem : 1)) == ESP_OK) {
            if (m.identifier == wantID) { out = m; return true; }
        } else {
            break;
        }
    }
    return false;
}

// ---------------------- Motor commands --------------------------------
// Send an absolute-position command; does not wait for settle.
// deg: target angle in degrees (centi-deg internally, matching motor protocol)
static void sendAbsPosition(uint8_t id, float deg, uint16_t maxDps) {
    int32_t ac = (int32_t)lroundf(deg * 100.0f);
    uint8_t tx[8] = {
        CMD_ABS_POSITION, 0x00,
        (uint8_t)maxDps,        (uint8_t)(maxDps >> 8),
        (uint8_t)ac,            (uint8_t)(ac >>  8),
        (uint8_t)(ac >> 16),    (uint8_t)(ac >> 24)
    };
    twai_send(TX_BASE + id, tx);
    twai_message_t rx;
    twai_recv(RX_BASE + id, rx, 15); // drain echo; ignore result
}

static void motorStop(uint8_t id) {
    uint8_t tx[8] = { CMD_MOTOR_STOP, 0,0,0,0,0,0,0 };
    twai_send(TX_BASE + id, tx);
}

// ---------------------- CAN / TWAI init -------------------------------
static inline twai_timing_config_t timing_from_hz(int hz) {
    switch (hz) {
        case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
        case 500000:  return TWAI_TIMING_CONFIG_500KBITS();
        case 250000:  return TWAI_TIMING_CONFIG_250KBITS();
        case 125000:  return TWAI_TIMING_CONFIG_125KBITS();
        default:      return TWAI_TIMING_CONFIG_500KBITS();
    }
}

void can_init() {
    twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
    twai_timing_config_t  t_config = timing_from_hz(CAN_HZ);
    twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

    if (twai_driver_install(&g_config, &t_config, &f_config) != ESP_OK) {
        Serial0.println("[CAN] driver_install failed");
        return;
    }
    if (twai_start() != ESP_OK) {
        Serial0.println("[CAN] start failed");
        return;
    }
    can_ok = true;
    Serial0.printf("[CAN] started at %d bit/s\n", CAN_HZ);
}

// ---------------------- Wi-Fi AP init ---------------------------------
static void ap_init() {
    WiFi.mode(WIFI_MODE_AP);
    esp_wifi_set_ps(WIFI_PS_NONE);

    if (!WiFi.softAPConfig(ap_ip, ap_gw, ap_netmask))
        Serial0.println("[AP] softAPConfig failed; continuing with default 192.168.4.1");

    if (WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 3)) {
        delay(100);
        ap_ok = true;
        Serial0.println("[AP] SoftAP started");
    } else {
        Serial0.println("[AP] SoftAP start FAILED");
    }
    IPAddress ip = WiFi.softAPIP();
    Serial0.printf("[AP] SSID: %s  PASS: %s  IP: %s\n", AP_SSID, AP_PASS, ip.toString().c_str());
}

// ---------------------- Arduino setup/loop ----------------------------
void setup() {
    Serial0.begin(115200);
    delay(100);
    Serial0.println("\n[BOOT] ESP32-C3 Gimbal Controller");

    led.begin();
    led.setBrightness(128);
    led.clear();
    led.show();

    ap_init();

    if (!sense.begin()) {
        Serial0.println("[ERROR] IMU init failed");
    } else {
        imu_ok = true;
        Serial0.println("[OK] IMU ready");
    }

    can_init();

    if (imu_ok && can_ok && ap_ok) {
        led.setPixelColor(0, led.Color(20, 255, 20)); // green
        led.show();
        Serial0.println("[LED] Green: system OK");
    } else {
        led.setPixelColor(0, led.Color(255, 0, 0)); // red
        led.show();
        Serial0.println("[LED] Red: init error");
    }
}

void loop() {
    static uint32_t t0 = millis();
    static float prev_roll_error = 0.0f;
    static float prev_pitch_error = 0.0f;
    static bool first_pd_sample = true;

    float roll = 0, pitch = 0, yaw = 0, acc = 0;
    if (sense.readOrientation(roll, pitch, yaw, acc)) {
        float dt = (millis() - t0) / 1000.0f;
        if (dt <= 0.0f) dt = SAMPLE_MS / 1000.0f;

        // Target is a level IMU, so error is opposite the measured tilt.
        float roll_error  = -roll;
        float pitch_error = -pitch;

        float roll_derivative = 0.0f;
        float pitch_derivative = 0.0f;
        if (!first_pd_sample) {
            roll_derivative  = (roll_error - prev_roll_error) / dt;
            pitch_derivative = (pitch_error - prev_pitch_error) / dt;
        }

        prev_roll_error = roll_error;
        prev_pitch_error = pitch_error;
        first_pd_sample = false;

        float roll_target  = (k_p * roll_error) + (k_d * roll_derivative);
        float pitch_target = (k_p * pitch_error) + (k_d * pitch_derivative);

        // Hard clamp — never exceed ±GIMBAL_MAX_DEG regardless of IMU reading.
        if (roll_target  >  GIMBAL_MAX_DEG) roll_target  =  GIMBAL_MAX_DEG;
        if (roll_target  < -GIMBAL_MAX_DEG) roll_target  = -GIMBAL_MAX_DEG;
        if (pitch_target >  GIMBAL_MAX_DEG) pitch_target =  GIMBAL_MAX_DEG;
        if (pitch_target < -GIMBAL_MAX_DEG) pitch_target = -GIMBAL_MAX_DEG;

        // Apply mechanical mounting offset so motor encoder zero aligns with level.
        roll_target  += ROLL_MOUNT_OFFSET_DEG;
        pitch_target += PITCH_MOUNT_OFFSET_DEG;

        sendAbsPosition(MOTOR_ROLL,  roll_target,  GIMBAL_MAX_DPS);
        sendAbsPosition(MOTOR_PITCH, 0, GIMBAL_MAX_DPS);

        Serial0.printf("IMU  roll=%+7.2f  pitch=%+7.2f  yaw=%+7.2f\n", roll, pitch, yaw);
        Serial0.printf("ERR  roll=%+7.2f  pitch=%+7.2f\n", roll_error, pitch_error);
        Serial0.printf("CMD  roll=%+7.2f  pitch=%+7.2f  Kp=%.3f  Kd=%.3f\n", roll_target, pitch_target, k_p, k_d);
        delay(50);
    } else {
        Serial0.println("[WARN] No IMU data — stopping motors");
        first_pd_sample = true;
        prev_roll_error = 0.0f;
        prev_pitch_error = 0.0f;
        motorStop(MOTOR_ROLL);
        motorStop(MOTOR_PITCH);
    }

    // 50 Hz loop pacing
    uint32_t now  = millis();
    uint32_t next = t0 + SAMPLE_MS;
    if (now < next) delay(next - now);
    t0 = next;
}
