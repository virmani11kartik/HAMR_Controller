// tests/test_shims.cpp
#include "Arduino.h"
#include "WiFi.h"
#include "WiFiUdp.h"

// One-definition globals used by odometry.h / ekf_localization.h
IPAddress remoteIP{};
unsigned int remotePort = 0;         // matches: extern unsigned int remotePort;

extern const int CPR = 2048;         // matches: extern const int CPR;
extern const int GEAR_RATIO = 1;     // matches: extern const int GEAR_RATIO;
extern const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO;  // matches: extern const int TICKS_PER_WHEEL_REV;

volatile long ticksL = 0;            // matches: extern volatile long ticksL;
volatile long ticksR = 0;            // matches: extern volatile long ticksR;

WiFiUDP udp;                         // single definition lives here
