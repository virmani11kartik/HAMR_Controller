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

// Pick one of: 1000000, 500000, 250000, 125000
static const int CAN_HZ = 1000000;   // 1 Mbit/s

// If you're bench-testing with only the ESP32 on the bus (no other node to ACK),
// you can temporarily use NO_ACK mode by uncommenting this:
// #define TWAI_USE_NO_ACK 1

// ---------------------- App timing ------------------------------------
static const uint16_t SAMPLE_MS = 100; // 10 Hz

// ---------------------- State flags -----------------------------------
bool can_ok = false;
bool imu_ok = false;
bool ap_ok  = false;

// ---------------------- Helpers ---------------------------------------
static inline void pack_rpy(float roll, float pitch, float yaw, uint8_t out[8]) {
  // scale deg -> centi-deg into int16, little-endian
  int16_t r = (int16_t)lroundf(roll  * 100.0f);
  int16_t p = (int16_t)lroundf(pitch * 100.0f);
  int16_t y = (int16_t)lroundf(yaw   * 100.0f);
  out[0] = (uint8_t)(r & 0xFF);
  out[1] = (uint8_t)((r >> 8) & 0xFF);
  out[2] = (uint8_t)(p & 0xFF);
  out[3] = (uint8_t)((p >> 8) & 0xFF);
  out[4] = (uint8_t)(y & 0xFF);
  out[5] = (uint8_t)((y >> 8) & 0xFF);
  out[6] = 0;
  out[7] = 0;
}

static inline twai_timing_config_t timing_from_hz(int hz) {
  switch (hz) {
    case 1000000: return TWAI_TIMING_CONFIG_1MBITS();
    case 500000:  return TWAI_TIMING_CONFIG_500KBITS();
    case 250000:  return TWAI_TIMING_CONFIG_250KBITS();
    case 125000:  return TWAI_TIMING_CONFIG_125KBITS();
    default:
      // Fallback to 500k if an unsupported rate was requested
      return TWAI_TIMING_CONFIG_500KBITS();
  }
}

static void ap_init() {
  WiFi.mode(WIFI_MODE_AP);
  esp_wifi_set_ps(WIFI_PS_NONE);

  if (!WiFi.softAPConfig(ap_ip, ap_gw, ap_netmask)) {
    Serial0.println("[AP] softAPConfig failed; continuing with default 192.168.4.1");
  }
  if (WiFi.softAP(AP_SSID, AP_PASS, 6, 0, 3)) {
    delay(100);
    ap_ok = true;
    Serial0.println("[AP] SoftAP started");
  } else {
    Serial0.println("[AP] SoftAP start FAILED");
  }
  IPAddress ip = WiFi.softAPIP();
  Serial0.printf("[AP] SSID: %s  PASS: %s  IP: %s\n", AP_SSID, AP_PASS, ip.toString().c_str());
  Serial0.println("[AP] Connect your Laptop and Raspberry Pi to this SSID.");
  Serial0.println("[AP] Then SSH from Laptop to Pi using the Pi's AP-assigned IP (e.g. ssh pi@192.168.50.10).");
}

void can_init() {
#ifdef TWAI_USE_NO_ACK
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NO_ACK);
#else
  twai_general_config_t g_config = TWAI_GENERAL_CONFIG_DEFAULT(CAN_TX_PIN, CAN_RX_PIN, TWAI_MODE_NORMAL);
#endif
  twai_timing_config_t  t_config = timing_from_hz(CAN_HZ);
  twai_filter_config_t  f_config = TWAI_FILTER_CONFIG_ACCEPT_ALL();

  esp_err_t err = twai_driver_install(&g_config, &t_config, &f_config);
  if (err != ESP_OK) {
    Serial0.printf("[CAN] driver_install failed: %d\n", (int)err);
    return;
  }
  err = twai_start();
  if (err != ESP_OK) {
    Serial0.printf("[CAN] start failed: %d\n", (int)err);
    return;
  }

  can_ok = true;
  Serial0.printf("[CAN] started at %d bit/s (%s)\n", CAN_HZ,
#ifdef TWAI_USE_NO_ACK
                 "NO_ACK"
#else
                 "NORMAL"
#endif
  );

  twai_status_info_t st;
  if (twai_get_status_info(&st) == ESP_OK) {
    Serial0.printf("[CAN] state=%d tx_err=%d rx_err=%d\n",
                   st.state, st.tx_error_counter, st.rx_error_counter);
  }
}

bool can_send_rpy(float roll, float pitch, float yaw) {
  uint8_t data[8];
  pack_rpy(roll, pitch, yaw, data);

  twai_message_t msg = {0};
  msg.identifier = 0x100;   // standard 11-bit ID for RPY
  msg.flags = 0;            // standard frame, data frame
  msg.data_length_code = 8;
  memcpy(msg.data, data, 8);

  esp_err_t err = twai_transmit(&msg, pdMS_TO_TICKS(10)); // 10 ms timeout
  if (err != ESP_OK) {
    Serial0.printf("[CAN] TX err: %d\n", (int)err);
    return false;
  }
  return true;
}

// ---------------------- Arduino setup/loop ----------------------------
void setup() {
  Serial0.begin(115200);
  delay(100);
  Serial0.println("\n[BOOT] ESP32-C3 IMU→CAN");

  led.begin();
  led.setBrightness(128);
  led.clear();
  led.show();

  ap_init();

  if (!sense.begin()) {
    Serial0.println("[ERROR] IMU init failed; continuing so CAN still starts");
  } else {
    imu_ok = true;
    Serial0.println("[OK] IMU ready");
  }

  can_init();

  if (imu_ok && can_ok && ap_ok) {
    led.setPixelColor(0, led.Color(20, 255, 20)); // green
    led.show();
    Serial0.println("[LED] Green LED ON: system OK");
  } else {
    led.setPixelColor(0, led.Color(255, 0, 0)); // red
    led.show();
    Serial0.println("[LED] Not all systems OK");
  }
}

void loop() {
  static uint32_t t0 = millis();
  static uint32_t last_stat = 0;

  float roll = 0, pitch = 0, yaw = 0, acc = 0;
  if (sense.readOrientation(roll, pitch, yaw, acc)) {
    Serial0.printf("RPY(deg): %.2f, %.2f, %.2f | acc(rad): %.3f\n", roll, pitch, yaw, acc);
    can_send_rpy(roll, pitch, yaw);
  } else {
    Serial0.println("[WARN] No IMU data");
  }

  // Print CAN status once per second
  uint32_t now_ms = millis();
  if (now_ms - last_stat > 1000) {
    last_stat = now_ms;
    twai_status_info_t st;
    if (twai_get_status_info(&st) == ESP_OK) {
      Serial0.printf("[CAN] state=%d tx_err=%d rx_err=%d txq=%d rxq=%d\n",
                     st.state, st.tx_error_counter, st.rx_error_counter,
                     st.msgs_to_tx, st.msgs_to_rx);
    }
  }

  // 10 Hz loop pacing
  uint32_t now = millis();
  uint32_t next = t0 + SAMPLE_MS;
  if (now < next) delay(next - now);
  t0 = next;
}
