#pragma once
#include <cstdint>
#include <cstdio>
#include <cstdarg>
#include <cmath>
#include <algorithm>
#include <random>
#include <string>

using std::uint8_t;
using std::uint16_t;
using std::uint32_t;

// ------ time ------
inline unsigned long __test_now_ms = 0;
inline unsigned long millis() { return __test_now_ms; }
inline void advance_millis(unsigned long ms){ __test_now_ms += ms; }

// ------ interrupts ------
inline void noInterrupts() {}
inline void interrupts() {}

// ------ Serial mock ------
struct __SerialMock {
  void println(const char* s){ std::printf("%s\n", s); }
  void println(float v){ std::printf("%f\n", v); }
  void printf(const char* fmt, ...){
    va_list args; va_start(args, fmt); std::vprintf(fmt, args); va_end(args);
  }
};
static __SerialMock Serial;

// ------ math helpers ------
#ifndef PI
#define PI 3.14159265358979323846
#endif

inline float constrainf(float x, float a, float b){
  if(x < a) return a;
  if(x > b) return b;
  return x;
}
// Use std::isfinite; don't redeclare a global symbol
using std::isfinite;

// Arduino-style global max()
template<typename T>
inline T max(T a, T b){ return std::max(a,b); }

// ------ Arduino random() overloads ------
inline void randomSeed(unsigned long) {
  // no-op for tests
}
inline long random(long max_exclusive){
  static std::mt19937 rng{1234567};
  std::uniform_int_distribution<long> dist(0, max_exclusive - 1);
  return dist(rng);
}
inline long random(long min_inclusive, long max_exclusive){
  static std::mt19937 rng{7654321};
  std::uniform_int_distribution<long> dist(min_inclusive, max_exclusive - 1);
  return dist(rng);
}
