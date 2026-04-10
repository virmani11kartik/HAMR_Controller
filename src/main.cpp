#include <Arduino.h>
#include <Adafruit_NeoPixel.h>
#include "imu_85.h"
#include "driver/twai.h"

// ---------------------- LED (M5Stamp C3 onboard) ----------------------
#define RGB_PIN 2
static Adafruit_NeoPixel led(1, RGB_PIN, NEO_GRB + NEO_KHZ800);

// ---------------------- IMU (I2C) -------------------------------------
static IMU_85 sense(1, 0); // SDA=GPIO1, SCL=GPIO0

// ---------------------- CAN (TWAI) pins & rate ------------------------
static const gpio_num_t CAN_TX_PIN = GPIO_NUM_19;
static const gpio_num_t CAN_RX_PIN = GPIO_NUM_18;
static const int        CAN_HZ     = 1000000; // 1 Mbit/s

// ---------------------- Motor protocol --------------------------------
static const uint8_t  MOTOR_ROLL  = 1;
static const uint8_t  MOTOR_PITCH = 2;
static const uint32_t TX_BASE     = 0x140;
static const uint32_t RX_BASE     = 0x240;

static const uint8_t CMD_SPEED_CONTROL = 0xA2;
static const uint8_t CMD_MOTOR_STOP    = 0x81;

// ---------------------- Gimbal speed limit ----------------------------
// Max speed the PID output can ever request (deg/s).
// Start conservative; raise once direction/response is confirmed.
static const float GIMBAL_MAX_DPS = 50.0f;

// ---------------------- PID gains -------------------------------------
// Roll
static const float ROLL_KP = 2.0f;
static const float ROLL_KI = 0.00f;
static const float ROLL_KD = 0.2f;

// Pitch
static const float PITCH_KP = 0.30f;
static const float PITCH_KI = 0.00f;
static const float PITCH_KD = 0.08f;

// ---------------------- Controller tuning constants -------------------
static const float DERIV_ALPHA        = 0.30f;  // EMA smoothing on derivative term
static const float IMU_ALPHA          = 0.3f;  // EMA smoothing on IMU angle input
static const float ERROR_DEADZONE_DEG = 0.30f;  // errors smaller than this are zeroed
static const float SETPOINT_RAMP_DPS  = 20.0f;  // max setpoint change rate (deg/s)
static const float MAX_INTEGRATOR     =  30.0f;
static const float MIN_INTEGRATOR     = -30.0f;

// Desired angles (0 = level)
static const float ROLL_SETPOINT_DEG  = 0.0f;
static const float PITCH_SETPOINT_DEG = 0.0f;

// Per-axis speed clamps (can be tightened independently)
static const float ROLL_MAX_DPS  = GIMBAL_MAX_DPS;
static const float PITCH_MAX_DPS = GIMBAL_MAX_DPS;

// Flip to -1.0f if an axis responds in the wrong direction
static const float ROLL_CMD_SIGN  =  1.0f;
static const float PITCH_CMD_SIGN =  1.0f;

// ---------------------- Loop timing -----------------------------------
static const uint16_t SAMPLE_MS   = 40; // 25 Hz control loop
static const uint8_t  PRINT_EVERY =  1; // serial print every N loops

// ---------------------- State flags -----------------------------------
static bool can_ok = false;
static bool imu_ok = false;

// ---------------------- Data structures -------------------------------
struct AxisController {
    // Gains
    float kp;
    float ki;
    float kd;
    // Configuration
    float setpoint_deg;
    float max_dps;
    float cmd_sign;
    // Runtime state (zero-initialised then seeded on first IMU reading)
    float filtered_measurement_deg;
    float smooth_setpoint_deg;
    float integrator;
    float prev_measurement_deg;
    float deriv_filtered;
    bool  seeded;
};

struct MotorStatus {
    float torque_A  = 0.0f;
    float speed_dps = 0.0f;
    float angle_deg = 0.0f;
    bool  valid     = false;
};

// ---------------------- TWAI helpers ----------------------------------
static bool twai_send_frame(uint32_t id, const uint8_t data[8]) {
    twai_message_t msg = {};
    msg.identifier       = id;
    msg.flags            = 0;
    msg.data_length_code = 8;
    memcpy(msg.data, data, 8);
    return twai_transmit(&msg, pdMS_TO_TICKS(5)) == ESP_OK;
}

// Drain one reply frame matching wantID; ignores frames for other IDs.
static bool twai_recv_frame(uint32_t wantID, twai_message_t &out, uint32_t timeout_ms = 20) {
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
// Send 0xA2 Speed Closed-Loop Control command.
//
// speed_dps  : desired shaft speed in deg/s (signed).
// Protocol   : speedControl is int32_t, unit = 0.01 dps/LSB
//              → wire value = speed_dps * 100
//
// The reply frame contains torque, speed, and angle, so this call
// also serves as the status read — no separate 0x9C poll needed.
static MotorStatus sendSpeedCommand(uint8_t id, float speed_dps) {
    int32_t sc = (int32_t)lroundf(speed_dps * 100.0f);
    uint8_t tx[8] = {
        CMD_SPEED_CONTROL, 0x00, 0x00, 0x00,
        (uint8_t)(sc),
        (uint8_t)(sc >>  8),
        (uint8_t)(sc >> 16),
        (uint8_t)(sc >> 24)
    };
    twai_send_frame(TX_BASE + id, tx);

    twai_message_t rx;
    if (!twai_recv_frame(RX_BASE + id, rx, 15) || rx.data[0] != CMD_SPEED_CONTROL) {
        return {}; // valid = false
    }

    MotorStatus s;
    // DATA[1] = temperature (int8_t, 1 °C/LSB) — available if needed
    s.torque_A  = (int16_t)(rx.data[2] | (rx.data[3] << 8)) * 0.01f; // 0.01 A/LSB
    s.speed_dps = (float)  (int16_t)(rx.data[4] | (rx.data[5] << 8)); // 1 dps/LSB
    s.angle_deg = (float)  (int16_t)(rx.data[6] | (rx.data[7] << 8)); // 1 deg/LSB
    s.valid     = true;
    return s;
}

static void motorStop(uint8_t id) {
    uint8_t tx[8] = { CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0 };
    twai_send_frame(TX_BASE + id, tx);
}

// ---------------------- CAN / TWAI init -------------------------------
static twai_timing_config_t timing_from_hz(int hz) {
    switch (hz) {
        case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
        case  500000: return TWAI_TIMING_CONFIG_500KBITS();
        case  250000: return TWAI_TIMING_CONFIG_250KBITS();
        case  125000: return TWAI_TIMING_CONFIG_125KBITS();
        default:      return TWAI_TIMING_CONFIG_500KBITS();
    }
}

static void can_init() {
    twai_general_config_t g_config =
        TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
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

// ---------------------- PID controller --------------------------------
static float stepTowards(float current, float target, float max_step) {
    float delta = target - current;
    if      (delta >  max_step) return current + max_step;
    else if (delta < -max_step) return current - max_step;
    return target;
}

// Seed controller state from the first real IMU reading to prevent
// a startup transient.
static void seedAxis(AxisController &axis, float measurement_deg) {
    axis.filtered_measurement_deg = measurement_deg;
    axis.smooth_setpoint_deg      = measurement_deg;
    axis.prev_measurement_deg     = measurement_deg;
    axis.deriv_filtered           = 0.0f;
    axis.integrator               = 0.0f;
    axis.seeded                   = true;
}

// Run one PID step. Returns a speed command in deg/s.
//
// The PID error (degrees off level) drives the speed demand directly:
//   large error  → fast correction
//   small error  → slow correction
//   zero error   → motor commanded to 0 dps (holds position via motor's
//                  own closed-loop speed control at 0)
//
// This is the correct architecture for a stabilising gimbal: the outer
// (attitude) loop runs here, and the motor's inner speed loop executes
// the commanded rate.
static float updateAxisSpeed(AxisController &axis, float measurement_deg, float dt) {
    if (!axis.seeded) {
        seedAxis(axis, measurement_deg);
    }

    // EMA filter on raw IMU angle
    axis.filtered_measurement_deg =
        IMU_ALPHA * measurement_deg + (1.0f - IMU_ALPHA) * axis.filtered_measurement_deg;

    // Soft-ramp the setpoint to avoid step-demand transients
    const float max_step = SETPOINT_RAMP_DPS * dt;
    axis.smooth_setpoint_deg =
        stepTowards(axis.smooth_setpoint_deg, axis.setpoint_deg, max_step);

    // Error with deadzone
    float error_deg = axis.smooth_setpoint_deg - axis.filtered_measurement_deg;
    if (fabsf(error_deg) < ERROR_DEADZONE_DEG) error_deg = 0.0f;

    // Integrator with anti-windup clamp
    axis.integrator += error_deg * dt;
    if (axis.integrator >  MAX_INTEGRATOR) axis.integrator =  MAX_INTEGRATOR;
    if (axis.integrator <  MIN_INTEGRATOR) axis.integrator =  MIN_INTEGRATOR;

    // Derivative on measurement (not error) to avoid setpoint-kick
    float raw_deriv =
        -(axis.filtered_measurement_deg - axis.prev_measurement_deg) /
         (dt > 0.001f ? dt : 0.001f);
    axis.deriv_filtered =
        DERIV_ALPHA * raw_deriv + (1.0f - DERIV_ALPHA) * axis.deriv_filtered;
    axis.prev_measurement_deg = axis.filtered_measurement_deg;

    // PID output → speed in deg/s
    float speed_dps =
        axis.kp * error_deg +
        axis.ki * axis.integrator +
        axis.kd * axis.deriv_filtered;

    // Apply direction sign and clamp to per-axis maximum
    speed_dps *= axis.cmd_sign;
    if (speed_dps >  axis.max_dps) speed_dps =  axis.max_dps;
    if (speed_dps < -axis.max_dps) speed_dps = -axis.max_dps;

    return speed_dps;
}

// ---------------------- Arduino setup ---------------------------------
void setup() {
    Serial0.begin(115200);
    delay(100);
    Serial0.println("\n[BOOT] ESP32-C3 Gimbal Controller (speed-control mode)");

    led.begin();
    led.setBrightness(128);
    led.clear();
    led.show();

    if (!sense.begin()) {
        Serial0.println("[ERROR] IMU init failed");
    } else {
        imu_ok = true;
        Serial0.println("[OK] IMU ready");
    }

    can_init();

    if (imu_ok && can_ok) {
        led.setPixelColor(0, led.Color(20, 255, 20)); // green — all systems go
        led.show();
        Serial0.println("[LED] Green: system OK");
    } else {
        led.setPixelColor(0, led.Color(255, 0, 0)); // red — init error
        led.show();
        Serial0.println("[LED] Red: init error — check IMU/CAN wiring");
    }

    // Print CSV header for serial plotter / logging
    Serial0.println(
        "TUNE,time_ms"
        ",roll_raw,roll_filt,roll_cmd_dps,roll_err"
        ",pitch_raw,pitch_filt,pitch_cmd_dps,pitch_err"
        ",roll_mot_angle,roll_mot_speed"
        ",pitch_mot_angle,pitch_mot_speed"
    );
}

// ---------------------- Arduino loop ----------------------------------
void loop() {
    static uint32_t t0       = millis();
    static uint32_t prev_ms  = millis();
    static uint32_t loop_seq = 0;

    static AxisController roll_axis = {
        ROLL_KP, ROLL_KI, ROLL_KD,
        ROLL_SETPOINT_DEG,
        ROLL_MAX_DPS,
        ROLL_CMD_SIGN,
        /*filtered_measurement_deg*/ 0.0f,
        /*smooth_setpoint_deg*/      0.0f,
        /*integrator*/               0.0f,
        /*prev_measurement_deg*/     0.0f,
        /*deriv_filtered*/           0.0f,
        /*seeded*/                   false
    };

    static AxisController pitch_axis = {
        PITCH_KP, PITCH_KI, PITCH_KD,
        PITCH_SETPOINT_DEG,
        PITCH_MAX_DPS,
        PITCH_CMD_SIGN,
        0.0f, 0.0f, 0.0f, 0.0f, 0.0f, false
    };

    float roll = 0.0f, pitch = 0.0f, yaw = 0.0f, acc = 0.0f;

    if (sense.readOrientation(roll, pitch, yaw, acc)) {
        uint32_t now_ms = millis();
        float dt = (now_ms - prev_ms) / 1000.0f;
        if (dt < 0.001f) dt = 0.001f;
        prev_ms = now_ms;

        // Compute speed demands
        float roll_dps  = updateAxisSpeed(roll_axis,  roll,  dt);
        float pitch_dps = updateAxisSpeed(pitch_axis, pitch, dt);

        // Send speed commands — reply gives feedback at no extra cost
        MotorStatus roll_fb  = sendSpeedCommand(MOTOR_ROLL,  roll_dps);
        MotorStatus pitch_fb = sendSpeedCommand(MOTOR_PITCH, 0);

        // Serial telemetry (CSV, compatible with Serial Plotter)
        if ((loop_seq++ % PRINT_EVERY) == 0) {
            float roll_err  = roll_axis.smooth_setpoint_deg  - roll_axis.filtered_measurement_deg;
            float pitch_err = pitch_axis.smooth_setpoint_deg - pitch_axis.filtered_measurement_deg;

            Serial0.printf(
                "TUNE,%lu"
                ",%.3f,%.3f,%.3f,%.3f"
                ",%.3f,%.3f,%.3f,%.3f"
                ",%.1f,%.1f"
                ",%.1f,%.1f\n",
                (unsigned long)now_ms,
                roll,  roll_axis.filtered_measurement_deg,  roll_dps,  roll_err,
                pitch, pitch_axis.filtered_measurement_deg, pitch_dps, pitch_err,
                roll_fb.valid  ? roll_fb.angle_deg  : 0.0f,
                roll_fb.valid  ? roll_fb.speed_dps  : 0.0f,
                pitch_fb.valid ? pitch_fb.angle_deg : 0.0f,
                pitch_fb.valid ? pitch_fb.speed_dps : 0.0f
            );
        }
    } else {
        // No IMU data — stop both motors immediately
        Serial0.println("[WARN] No IMU data — stopping motors");
        motorStop(MOTOR_ROLL);
        motorStop(MOTOR_PITCH);
    }

    // Fixed-rate loop pacing at SAMPLE_MS (25 Hz)
    uint32_t now  = millis();
    uint32_t next = t0 + SAMPLE_MS;
    if (now < next) delay(next - now);
    t0 = next;
}