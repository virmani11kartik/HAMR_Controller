#pragma once
#include <cstdint>

struct IPAddress {
  uint32_t raw{0};
  IPAddress() = default;
  explicit IPAddress(uint32_t r): raw(r) {}
};

struct WiFiClass {
  enum Mode { WIFI_STA = 1 };
  void mode(Mode) {}
};
static WiFiClass WiFi;
