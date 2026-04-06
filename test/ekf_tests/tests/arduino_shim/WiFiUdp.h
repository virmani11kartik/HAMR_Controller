#pragma once
#include <cstddef>
#include "WiFi.h"   // for IPAddress

struct WiFiUDP {
  int begin(int){ return 1; }
  int beginPacket(const char*, int){ return 1; }
  int beginPacket(IPAddress, int){ return 1; }  // overload used by your code
  int write(const unsigned char*, size_t){ return 1; }
  int write(const char*, size_t){ return 1; }
  int print(const char*){ return 1; }          // Arduino-like print
  int endPacket(){ return 1; }
};
