
// licensed under the University of Pennsylvania, Version 1.0 (the "License");
// Kartik Virmani MODLAB-UPENN
// you may not use this file except in compliance with the License.

#include <Arduino.h>
#include <esp32-hal.h>
#include <esp32-hal-gpio.h>
#include <esp32-hal-ledc.h>
#include <WiFi.h>
#include <WiFiUdp.h>
#include <WebServer.h>
#include <Wire.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_BNO055.h>
#include <Adafruit_BNO08x.h>
#include "odometry.h"
#include <string.h>
#include "pid_webpage.h"
#include "imu_55.h"
#include "imu_85.h"
#include "ekf_localization.h"

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

//------------IMU OBJECT--------------
IMU55 sens(33,34);                 // SDA=4, SCL=5, addr=0x28 by default
float roll_b,pitch_b,yaw_b;
//------------EKF CONFIG--------------
EkfYawConfig cfg;

//---------------------------GLOBALS----------------------
// UDP SETUP
WiFiUDP udp;
const int port = 12345;  // Port to listen on
char incoming[256];  // Buffer for incoming data
IPAddress remoteIP;
unsigned int remotePort;
// Left
const int pwmL = 11;
const int dirL = 12;
const int encAL = 14;
const int encBL = 13;
// Right
const int pwmR = 41;
const int dirR = 42;
const int encAR = 1;
const int encBR = 2;
// Turret
const int pwmT = 7;  
const int dirT = 6;   
const int enAT = 5;
const int enBT = 4;

// Encoder counts (volatile for ISR)
volatile long ticksL = 0;
volatile long ticksR = 0;
volatile long ticksT = 0;

// PID constants for synchronization
float Kp_L = 120.0f, Ki_L = 40.0f, Kd_L = 0.0f;   // tune per wheel
float Kp_R = 120.0f, Ki_R = 40.0f, Kd_R = 0.0f;   // tune per wheel

// Encoder & motor specs
const int CPR = 64;
const int GEAR_RATIO = 150;
const int TICKS_PER_WHEEL_REV = CPR * GEAR_RATIO; // 9600 ticks per wheel revolution

// Turret motor specs
const int TICKS_PER_TURRET_REV = 2704; // 13 PPR × 2 (quadrature) × 104 (gear ratio) = 2704 ticks/rev at output
const float DEGREES_PER_TURRET_TICK = 360.0 / TICKS_PER_TURRET_REV; // Degrees per tick
const float motorGearTeeth = 40.0; // Motor gear teeth
const float outputGearTeeth = 136.0; // Output gear teeth
const float turretGearRatio = outputGearTeeth / motorGearTeeth; // Turret gear ratio

// Target angle for turret in degrees
float currentAngleT = 0.0;
float inputTurretAngle = 0.0;  // Desired turret angle in degrees
float targetTurretAngle = inputTurretAngle * turretGearRatio; // Target angle in degrees

// PID constants for turret control
const float Kp_turret = 18.0;
const float Ki_turret = 0.05;
const float Kd_turret = 0.1;

float integralT = 0.0; // Integral term for turret PID0
float lastErrorT = 0.0; // Last error for turret PID
unsigned long lastTurretTime = 0; // Last time turret PID was updated
// PWM limits
const int minPWM = 200;   // Minimum PWM value
const int maxPWM = 4095; // Maximum PWM value (12-bit resolution)
const int maxPWM_D = 4095;
float pwmL_out = 0.0;
float pwmR_out = 0.0;

// Control interval (ms)
const unsigned long PID_INTERVAL = 10;
static unsigned long lastUdpTime = 0;
static unsigned long lastSerialCtlTime = 0;

// PID state variables
float integralL = 0.0f, integralR = 0.0f;
float lastErrL = 0.0f, lastErrR = 0.0f;
float errorT = 0;

// Timing variables
unsigned long lastPidTime = 0;
long lastTicksL = 0;
long lastTicksR = 0;
long lastTicksT = 0;

// Base PWM speed (0-4095)
float basePWM = 3500;
const float MAX_RPM_CMD = 28.0f;
float pwmT_out = 0;
float scaleFactor = 1.0; //131.67;
// float turretSpeed = 0.0;

float test = 0.0f;

// Joystick control variables
float ly = 0.0f;  // left stick vertical (forward/back)
float rx = 0.0f;  // right stick horizontal (turn)
float lt = INT32_MIN; // left trigger
float rt = INT32_MIN; // left/right triggers
// HTTML Joystick control variables
float joyX = 0.0f;  // Joystick X-axis
float joyY = 0.0f;  // Joystick Y-axis
float joyturretX = 0.0f;  // Turret joystick X-axis
float joyturretY = 0.0f;  // Turret joystick Y-axis
String btn = "stop"; // Button pressed (e.g., 'f' for forward, 'b' for backward)
float value = 0.0f; // Value of the button pressed

// Odometry timing
unsigned long lastOdometryTime = 0;
const unsigned long ODOMETRY_INTERVAL = 100; // Odometry update interval in ms

// ----------------- UART Protocol -----------------
static const uint16_t MAGIC = 0xCAFE;
static const uint16_t VER   = 1;
static const uint16_t TYPE_CMD  = 0x0001; // PC->ESP : left,right
static const uint16_t TYPE_CMD3 = 0x0011; // PC->ESP : left,right,turret
static const uint16_t TYPE_ENC  = 0x0003; // ESP->PC : encoders
static const uint16_t TYPE_POSE = 0x0004; // ESP->PC : pose (x,y,theta) + uncertainty
// Latest commands received over UART (ROS)
volatile float uart_left_cmd = 0.0f;
volatile float uart_right_cmd = 0.0f;
volatile float uart_turret_cmd = 0.0f;
volatile uint32_t last_uart_cmd_ms = 0;
static uint32_t pose_seq = 0;

// Enc packet sequence
static uint32_t enc_seq = 0;

#pragma pack(push,1)
struct CmdPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right;
  uint16_t crc16;
};
struct Cmd3Packet {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float left, right, turret;
  uint16_t crc16;
};
struct EncPacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  int32_t ticksL, ticksR, ticksT;
  uint16_t crc16;
};
struct PosePacket {
  uint16_t magic, ver, type;
  uint32_t seq;
  uint64_t t_tx_ns;
  float x, y, theta;           // Robot pose in meters and radians
  float sigma_x, sigma_y, sigma_theta; // Uncertainties
  uint8_t ekf_status;          // 0=odometry_only, 1=ekf_fused, 2=imu_invalid
  uint16_t crc16;
};
#pragma pack(pop)

static const size_t CMD_SIZE  = sizeof(CmdPacket);   // 2-float
static const size_t CMD3_SIZE = sizeof(Cmd3Packet);  // 3-float
static const size_t ENC_SIZE  = sizeof(EncPacket);
static const size_t POSE_SIZE = sizeof(PosePacket);

// CRC32->16 surrogate (must match Pi side)
uint16_t crc16_surrogate(const uint8_t* data, size_t n) {
  uint32_t c = 0xFFFFFFFFu;
  for (size_t i=0;i<n;i++) {
    c ^= data[i];
    for (int k=0;k<8;k++) {
      c = (c & 1) ? (0xEDB88320u ^ (c >> 1)) : (c >> 1);
    }
  }
  c ^= 0xFFFFFFFFu;
  return (uint16_t)(c & 0xFFFF);
}

void transmitPoseData() {
    PosePacket pose;
    pose.magic = MAGIC; 
    pose.ver = VER; 
    pose.type = TYPE_POSE;
    pose.seq = ++pose_seq;
    pose.t_tx_ns = (uint64_t)micros() * 1000ull;
    
    // Get current pose (after EKF fusion)
    pose.x = getRobotX();
    pose.y = getRobotY();
    pose.theta = getRobotTheta();
    
    // Get uncertainties (standard deviations)
    pose.sigma_x = getUncertaintyX();
    pose.sigma_y = getUncertaintyY();
    pose.sigma_theta = getUncertaintyTheta();
    
    // Set EKF status flag
    if (sens.getStatus() == IMU_OK && sens.isDataValid()) {
        pose.ekf_status = 1; // EKF fused
    } else if (sens.getStatus() != IMU_INIT_FAILED) {
        pose.ekf_status = 2; // IMU available but invalid/uncalibrated
    } else {
        pose.ekf_status = 0; // Odometry only
    }
    
    pose.crc16 = crc16_surrogate((uint8_t*)&pose, POSE_SIZE - 2);
    
    Serial0.write((uint8_t*)&pose, POSE_SIZE);
}

// ------------- Units & conversion -------------
constexpr float WHEEL_RADIUS_M = 0.0762f;      // your wheel radius
constexpr float MAX_WHEEL_RPM  = 30.0f;       // safety clamp (tune)
enum WheelCmdUnits { CMD_MPS, CMD_RAD_PER_S, CMD_RPM };
constexpr WheelCmdUnits CMD_UNITS = CMD_RAD_PER_S;   // ROS cmd units

inline float wheel_rpm_from_cmd(float v) {
  switch (CMD_UNITS) {
    case CMD_MPS:      return v * 60.0f / (2.0f * (float)M_PI * WHEEL_RADIUS_M);
    case CMD_RAD_PER_S:return v * 60.0f / (2.0f * (float)M_PI);
    case CMD_RPM:
    default:           return v;
  }
}
template<typename T> inline T clamp(T x, T lo, T hi) {
  return x < lo ? lo : (x > hi ? hi : x);
}

// ------------------------ ENCODERS------------------
// Encoder interrupts (quadrature decoding)
void IRAM_ATTR handleEncL() {
  bool A = digitalRead(encAL);
  bool B = digitalRead(encBL);
  ticksL += (A == B) ? 1 : -1;
}

void IRAM_ATTR handleEncR() {
  bool A = digitalRead(encAR);
  bool B = digitalRead(encBR);
  ticksR += (A == B) ? -1 : 1;  // Inverted to fix direction issue if needed
}

// Turret encoder interrupt
void IRAM_ATTR handleEncT() {
  bool A = digitalRead(enAT);
  bool B = digitalRead(enBT);
  ticksT += (A == B) ? 1 : -1; // Adjust based on your encoder wiring
}

//-----------------------------SET MOTORS---------------------
// Set motor PWM and direction
void setMotor(int pwmPin, int dirPin, float pwmVal, int channel) {
  pwmVal = constrain(pwmVal, -4095, 4095);
  if (pwmVal >= 0) {
    digitalWrite(dirPin, HIGH);
    ledcWrite(channel, (int)pwmVal);
  } else {
    digitalWrite(dirPin, LOW);
    ledcWrite(channel, (int)(-pwmVal));
  }
}

//---------------------------UDP TRANSMISSION (IF WIFI)-------------
void sendUDP(String msg) {
  if (remoteIP && remotePort) {
    udp.beginPacket(remoteIP, remotePort);
    udp.print(msg);
    udp.endPacket();
  }
}

//---------------------------ODOM SET---------------------------
void setupProbabilisticEndpoints() {
  // Endpoint to get current pose with uncertainty
  server.on("/pose", HTTP_GET, []() {
    String json = "{";
    json += "\"x\":" + String(getRobotX(), 6) + ",";
    json += "\"y\":" + String(getRobotY(), 6) + ",";
    json += "\"theta\":" + String(getRobotTheta(), 6) + ",";
    json += "\"uncertainty_x\":" + String(getUncertaintyX(), 6) + ",";
    json += "\"uncertainty_y\":" + String(getUncertaintyY(), 6) + ",";
    json += "\"uncertainty_theta\":" + String(getUncertaintyTheta(), 6);
    json += "}";
    server.send(200, "application/json", json);
  });
  
  // Endpoint to reset odometry
  server.on("/reset", HTTP_GET, []() {
    resetOdometry();
    server.send(200, "text/plain", "Odometry reset");
  });
  
  // Endpoint to sample from pose distribution
  server.on("/sample", HTTP_GET, []() {
    float sample_x, sample_y, sample_theta;
    samplePose(sample_x, sample_y, sample_theta);
    String json = "{";
    json += "\"sample_x\":" + String(sample_x, 6) + ",";
    json += "\"sample_y\":" + String(sample_y, 6) + ",";
    json += "\"sample_theta\":" + String(sample_theta, 6);
    json += "}";
    server.send(200, "application/json", json);
  });
}

//-----------------------------------ESP SETUP-------------------------
void setup() {

  Serial.begin(115200);   // USB CDC logs
  Serial0.begin(460800);
  Serial.println("ESP32 bidirectional UART Ready");
  Serial.println("Pulling Micro-ROS");

  // ----------------Initialize odometry--------------
  initOdometry(); 
  cfg.R_yaw_rad2 = sq(12.0f * M_PI / 180.0f);
  cfg.gate_sigma = 3.0f;  // set <=0 to disable gating
  cfg.alignment_timeout_ms = 5000.0f;
  cfg.min_calibration_level = 2;
  cfg.enable_periodic_realignment = true;
  cfg.realignment_threshold = 30.0f * M_PI / 180.0f; // 30 degrees
  cfg.realignment_count_threshold = 10;

  // ----------------WIFI SETUP----------------------------------------
  WiFi.softAP(ssid, password, 4, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  //--------------------------------ESP SERVER---------------------------
  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", pid_webpage);  // Serve the HTML page
  });
  
  server.on("/move", HTTP_GET, []() {
    float x = server.arg("x").toFloat();
    float y = server.arg("y").toFloat();
    joyX = x;
    joyY = -y;
    server.send(200, "text/plain", "Movement received");
  });

// Trigger buttons: LT or RT
  server.on("/trigger", HTTP_GET, []() {
    btn = server.arg("btn");
    value = server.arg("value").toFloat();  // Use 'value' param
    server.send(200, "text/plain", "Trigger received: " + btn);
  });

// Turret angle input
  server.on("/setTurretAngle", HTTP_GET, []() {
    float angle = server.arg("angle").toFloat();
    inputTurretAngle = -angle;
    targetTurretAngle = inputTurretAngle * turretGearRatio; // Apply gear ratio
    server.send(200, "text/plain", "Turret angle set.");
  });

  // GET current values (already suggested)
  server.on("/getPID", HTTP_GET, []() {
    String json = "{";
    json += "\"Kp\":" + String(Kp_R) + ",";
    json += "\"Ki\":" + String(Ki_R) + ",";
    json += "\"Kd\":" + String(Kd_R) + ",";
    json += "\"Test\":" + String(test, 4); // 4 decimals for float
    json += "}";
    server.send(200, "application/json", json);
  });

  // POST updates
  server.on("/updatePID", HTTP_POST, []() {
    if (server.hasArg("Kp")) Kp_R = server.arg("Kp").toFloat();
    if (server.hasArg("Ki")) Ki_R = server.arg("Ki").toFloat();
    if (server.hasArg("Kd")) Kd_R = server.arg("Kd").toFloat();
    if (server.hasArg("Test")) test = server.arg("Test").toFloat();
    String response = "Updated PID values:\nKp=" + String(Kp_L) + "\nKi=" + String(Ki_L) + "\nKd=" + String(Kd_L) +
                      "\nTest=" + String(test, 4);
    server.send(200, "text/plain", response);
  });

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });
  server.begin();
  Serial.println("HTTP server started");
  udp.begin(port);
  Serial.printf("Listening for UDP on port %d\n", port);

  //---------------------------------PIN DEFINITIONS------------------------
  // Motor pins
  pinMode(pwmL, OUTPUT);
  pinMode(dirL, OUTPUT);
  pinMode(pwmR, OUTPUT);
  pinMode(dirR, OUTPUT);

  // Turret motor pins
  pinMode(pwmT, OUTPUT);
  pinMode(dirT, OUTPUT);

  // Setup PWM channels at 5 kHz, 12-bit resolution
  ledcSetup(0, 5000, 12);
  ledcSetup(1, 5000, 12);
  ledcSetup(2, 5000, 12); // Turret PWM channel
  ledcAttachPin(pwmL, 0);
  ledcAttachPin(pwmR, 1);
  ledcAttachPin(pwmT, 2); // Attach turret PWM pin to channel 2

  // Encoder pins with pull-ups
  pinMode(encAL, INPUT_PULLUP);
  pinMode(encBL, INPUT_PULLUP);
  pinMode(encAR, INPUT_PULLUP);
  pinMode(encBR, INPUT_PULLUP);
  // Turret encoder pins with pull-ups
  pinMode(enAT, INPUT_PULLUP);
  pinMode(enBT, INPUT_PULLUP);
  
  // Attach interrupts on channel A for both encoders
  attachInterrupt(digitalPinToInterrupt(encAL), handleEncL, CHANGE);
  attachInterrupt(digitalPinToInterrupt(encAR), handleEncR, CHANGE);
  // Attach turret encoder interrupt
  attachInterrupt(digitalPinToInterrupt(enAT), handleEncT, CHANGE);

  lastPidTime = millis();
  lastTurretTime = millis();

  Serial.println("Low Level Control Ready");
  Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);

  ///=========== IMU_BASE SETUP =========////
  // while (!Serial) {}
  // if (!sens.begin()) {
  //   Serial.println("IMU init failed. Check wiring/address.");
  //   while (1) delay(1000);
  // }
  // sens.setMinCalibrationLevel(2); 
  // sens.setDataTimeout(1000);
}

void loop() {

  //=======IMU_BASE READ=======
    // sens.update();
    // sens.getRPY(roll_b,pitch_b,yaw_b);
    // Serial.println(sens.isCalibrated() ? "Fully Calibrated" : "Not Calibrated");
    // Serial.printf("Roll=%.2f Pitch=%.2f Yaw=%.2f\n", roll_b,pitch_b,yaw_b);

  //-------------------------MICRO_ROS_PROTCOL-------------------------------
  // ---- RX: parse commands (robust to CMD or CMD3) ----
  static uint8_t buf[64];      // big enough for CMD3 (48 bytes) + slack
  static size_t  have = 0;

    // accumulate
  while (Serial0.available() && have < sizeof(buf)) {
      buf[have++] = (uint8_t)Serial0.read();
    }
    // try to parse while we have at least a header
  while (have >= 6) { // magic(2)+ver(2)+type(2)
      uint16_t magic = (uint16_t)buf[0] | ((uint16_t)buf[1] << 8);
      if (magic != MAGIC) {
        // resync: drop 1 byte
        memmove(buf, buf+1, --have);
        continue;
      }
      uint16_t ver  = (uint16_t)buf[2] | ((uint16_t)buf[3] << 8);
      uint16_t type = (uint16_t)buf[4] | ((uint16_t)buf[5] << 8);

      if (ver != VER) {
        memmove(buf, buf+1, --have);
        continue;
      }
      size_t need = (type == TYPE_CMD) ? CMD_SIZE :
                  (type == TYPE_CMD3)? CMD3_SIZE : 0;
      if(need==0){
        // unknown type; drop 1 byte
        memmove(buf, buf+1, --have);
        continue;
      }
      if (have < need) break; // incomplete frame
      if (type == TYPE_CMD && need == CMD_SIZE) {
        CmdPacket cmd; memcpy(&cmd, buf, CMD_SIZE);
        uint16_t calc = crc16_surrogate((uint8_t*)&cmd, CMD_SIZE - 2);
      if (calc == cmd.crc16) {
        noInterrupts();
        uart_left_cmd  = cmd.left;
        uart_right_cmd = cmd.right;
        last_uart_cmd_ms = millis();
        interrupts();
        // (optional) Serial.printf("UART CMD: L=%.3f R=%.3f\n", cmd.left, cmd.right);
      }
      memmove(buf, buf + CMD_SIZE, have - CMD_SIZE); have -= CMD_SIZE;
    } else if (type == TYPE_CMD3 && need == CMD3_SIZE) {
      Cmd3Packet cmd3; memcpy(&cmd3, buf, CMD3_SIZE);
      uint16_t calc = crc16_surrogate((uint8_t*)&cmd3, CMD3_SIZE - 2);
      if (calc == cmd3.crc16) {
        noInterrupts();
        uart_left_cmd   = cmd3.left * -1;
        uart_right_cmd  = cmd3.right * -1;
        uart_turret_cmd = cmd3.turret;
        last_uart_cmd_ms = millis();
        interrupts();
        // (optional) Serial.printf("UART CMD3: L=%.3f R=%.3f T=%.3f\n", cmd3.left, cmd3.right, cmd3.turret);
      }
      memmove(buf, buf + CMD3_SIZE, have - CMD3_SIZE); have -= CMD3_SIZE;
    } else {
      // shouldn’t happen
      memmove(buf, buf+1, --have);
    }
  }
    
  //-----------------------TELEOP PROTOCOL------------------------------
  // Handle serial commands, Read joystick data from serial

  // Serial.setTimeout(5);
  if (Serial.available()) {          
    String msg = Serial.readStringUntil('\n');
    msg.trim();

    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");
    if (lyIndex < 0 || rxIndex < 0 || ltIndex < 0 || rtIndex < 0);

    auto seg = [&](int idx)->String {
      int end = msg.indexOf(' ', idx);
      if (end < 0) end = msg.length();
      return msg.substring(idx + 3, end);
    };

    ly = seg(lyIndex).toFloat();
    rx = seg(rxIndex).toFloat();
    lt = seg(ltIndex).toFloat();
    rt = seg(rtIndex).toFloat();
    lastSerialCtlTime = millis();
  }

  server.handleClient(); // Handle HTTP requests
  int len = udp.parsePacket();
  if (len > 0) {
    udp.read(incoming, sizeof(incoming));
    incoming[len] = '\0';  // null-terminate
    // Serial.printf("Received: %s\n", incoming);
    String msg = String(incoming); 

    remoteIP = udp.remoteIP();
    remotePort = udp.remotePort();

    int lyIndex = msg.indexOf("LY:");
    int rxIndex = msg.indexOf("RX:");
    int ltIndex = msg.indexOf("LT:");
    int rtIndex = msg.indexOf("RT:");

  if (lyIndex != -1 && rxIndex != -1 && ltIndex != -1 && rtIndex != -1) {
    // Extract LY and RX values as floats
    // Find next space or end of line after LY:
    int lyEnd = msg.indexOf(' ', lyIndex);
    if (lyEnd == -1) lyEnd = msg.length();
    int rxEnd = msg.indexOf(' ', rxIndex);
    if (rxEnd == -1) rxEnd = msg.length();
    int ltEnd = msg.indexOf(' ', ltIndex);
    if (ltEnd == -1) ltEnd = msg.length();
    int rtEnd = msg.indexOf(' ', rtIndex);
    if (rtEnd == -1) rtEnd = msg.length();

    String lyStr = msg.substring(lyIndex + 3, lyEnd);
    String rxStr = msg.substring(rxIndex + 3, rxEnd);
    String ltStr = msg.substring(ltIndex + 3, ltEnd);
    String rtStr = msg.substring(rtIndex + 3, rtEnd);

    ly = lyStr.toFloat();
    rx = rxStr.toFloat();
    lt = ltStr.toFloat();
    rt = rtStr.toFloat();
    lastUdpTime = millis();
    }
  }

  ////=======================PID loop timing==================////
  unsigned long now = millis();
  if (now - lastPidTime >= PID_INTERVAL) {
    float dt = (now - lastPidTime) / 1000.0;

    ////=================== DRIVE CONTROL =================////
    // Read encoder counts atomically
    noInterrupts();
    long currentTicksL = ticksL;
    long currentTicksR = ticksR;
    long currentTicksT = ticksT;
    interrupts();

    // Calculate RPM for each motor
    float rpmL = ((currentTicksL - lastTicksL) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
    float rpmR = ((currentTicksR - lastTicksR) / dt) * 60.0 / TICKS_PER_WHEEL_REV;
    float rpmT = ((currentTicksT - lastTicksT) / dt) * 60.0 / TICKS_PER_TURRET_REV;
    float rpmT_platform = rpmT * turretGearRatio; // Platform RPM

    lastTicksL = currentTicksL;
    lastTicksR = currentTicksR;
    lastPidTime = now;

    // Calculate ticks and pose send to Micro ROS Bridge to Publish 
    static uint32_t last_tx_ms = 0;
    if (millis() - last_tx_ms >= 10) {
      last_tx_ms = millis();

      EncPacket enc;
      enc.magic = MAGIC; enc.ver = VER; enc.type = TYPE_ENC;
      enc.seq = ++enc_seq;
      enc.t_tx_ns = (uint64_t)micros() * 1000ull;
      noInterrupts(); enc.ticksL = ticksL; enc.ticksR = ticksR; enc.ticksT = ticksT; interrupts();
      enc.crc16 = crc16_surrogate((uint8_t*)&enc, ENC_SIZE - 2);

      Serial0.write((uint8_t*)&enc, ENC_SIZE); // binary out on the data UART
    }
    static uint32_t last_pose_tx_ms = 0;
    if (millis() - last_pose_tx_ms >= 10) {
        last_pose_tx_ms = millis();
        transmitPoseData();
    }

    bool useUdp = (millis() - lastUdpTime < 100);
    bool useUart = (millis() - last_uart_cmd_ms < 100);
    bool useSerialCtl = (millis() - lastSerialCtlTime  < 1000);

    float rpmTargetL = 0.0f, rpmTargetR = 0.0f;
    float pwmFF_L = 0.0f, pwmFF_R = 0.0f;
    
    if (useUart) {
      // Convert ROS cmd vel -> wheel RPM, then clamp to your controller’s max
      float Lrpm = clamp(wheel_rpm_from_cmd(uart_left_cmd),  -MAX_RPM_CMD,  MAX_RPM_CMD);
      float Rrpm = clamp(wheel_rpm_from_cmd(uart_right_cmd), -MAX_RPM_CMD,  MAX_RPM_CMD);
      rpmTargetL = Lrpm;
      rpmTargetR = Rrpm;

      // Keep your existing FF style: scale basePWM by the normalized target
      pwmFF_L = basePWM * (rpmTargetL / MAX_RPM_CMD);
      pwmFF_R = basePWM * (rpmTargetR / MAX_RPM_CMD);
    }
    else if (useSerialCtl) {
      float forward = ly * 0.8f;
      float turn    = rx * 0.8f;
      rpmTargetL = (forward + turn) * MAX_RPM_CMD;
      rpmTargetR = (forward - turn) * MAX_RPM_CMD;
      pwmFF_L = (forward + turn) * basePWM;
      pwmFF_R = (forward - turn) * basePWM;
    } else {
      // UDP / HTML fallback
      float forward = useUdp ? ly : joyY;
      float turn    = useUdp ? rx : joyX;
      forward *= 0.8f; turn *= 0.8f;
      rpmTargetL = (forward + turn) * MAX_RPM_CMD;
      rpmTargetR = (forward - turn) * MAX_RPM_CMD;
      pwmFF_L = (forward + turn) * basePWM;
      pwmFF_R = (forward - turn) * basePWM;
    }

    // Calculate error in motord
    float errL = rpmTargetL - rpmL;
    float errR = rpmTargetR - rpmR;
    
    bool satL = (pwmL_out >=  maxPWM_D) || (pwmL_out <= -maxPWM_D);
    bool satR = (pwmR_out >=  maxPWM_D) || (pwmR_out <= -maxPWM_D);
    if (!satL) integralL += errL * dt;
    if (!satR) integralR += errR * dt;

    // Clamp integrators a bit
    integralL = constrain(integralL, -300.0f, 300.0f);
    integralR = constrain(integralR, -300.0f, 300.0f);
    float dErrL = (errL - lastErrL) / dt;
    float dErrR = (errR - lastErrR) / dt;
    lastErrL = errL;
    lastErrR = errR;

    float deltaPWM_L = Kp_L * errL + Ki_L * integralL + Kd_L * dErrL;
    float deltaPWM_R = Kp_R * errR + Ki_R * integralR + Kd_R * dErrR;

    // 4) Final PWM = feed-forward + PID correction, with saturation & deadband
    pwmL_out = constrain(pwmFF_L + deltaPWM_L, -maxPWM_D, maxPWM_D);
    pwmR_out = constrain(pwmFF_R + deltaPWM_R, -maxPWM_D, maxPWM_D);

    if (fabsf(pwmL_out) < minPWM) pwmL_out = 0.0f;
    if (fabsf(pwmR_out) < minPWM) pwmR_out = 0.0f;

    ////=================== TURRET CONTROL =================////
    if (btn == "l") {
      pwmT_out = -value * maxPWM;
    } else if (btn == "r") {
      pwmT_out = value * maxPWM;
    } else if (btn == "stop") {
      pwmT_out = 0;
    }
    setMotor(pwmT, dirT, pwmT_out, 2);
    
    if (useUart) {
      // ===== Open-loop turret drive: cmd [rad/s] -> PWM =====
      // 1) clamp incoming command to a safe range
      const float MAX_CMD_RAD_S = 2.0f;                // tune: your max platform speed
      float cmd = constrain(uart_turret_cmd, -MAX_CMD_RAD_S, MAX_CMD_RAD_S);

      // 2) deadband for tiny commands
      const float CMD_DEADBAND = 0.05f;                // rad/s (ignore tiny commands)
      if (fabsf(cmd) < CMD_DEADBAND) cmd = 0.0f;

      // 3) map |cmd| -> PWM with static friction compensation
      //    - PWM_STATIC: minimal PWM to break static friction
      //    - remaining range scales linearly with |cmd|
      const float PWM_STATIC = 350.0f;                 // tune: 250–600 typical
      float pwm_mag = 0.0f;
      if (cmd != 0.0f) {
        float frac = fabsf(cmd) / MAX_CMD_RAD_S;       // 0..1
        pwm_mag = PWM_STATIC + frac * (maxPWM - PWM_STATIC);
      }

      float pwm_target = copysignf(pwm_mag, cmd);

      // 4) slew limit to avoid jerks
      static float pwm_prev = 0.0f;
      const float PWM_SLEW_PER_S = 8000.0f;            // tune: PWM units per second
      float max_step = PWM_SLEW_PER_S * dt;
      float step = constrain(pwm_target - pwm_prev, -max_step, max_step);
      pwm_prev += step;

      // 5) apply limits and output
      pwmT_out = constrain(pwm_prev, -maxPWM, maxPWM);

      // Optional: status (platform angle, since ticks are output-side)
      currentAngleT = fmodf(ticksT * DEGREES_PER_TURRET_TICK, 360.0f);
    }

    if (lt > 0.1f || rt > 0.1f){
    // Simple turret control based on triggers
      targetTurretAngle = 0.0;
      float turretSpeed = (lt > 0.1) ? -lt * maxPWM : rt * maxPWM;
      pwmT_out = turretSpeed;
      currentAngleT = ticksT * DEGREES_PER_TURRET_TICK; // Calculate turret angle in degrees
      currentAngleT = fmod(currentAngleT/turretGearRatio, 360.0);
    }
    else if (fabs(targetTurretAngle)>0.0f) {
      // Turret PID control
      // Read turret encoder counts atomically
      noInterrupts();
      long currentTicksT = ticksT;
      interrupts();
      // Calculate turret angle in degrees
      float currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
      // Calculate error for turret PID
      float errorT = targetTurretAngle - currentAngleT;
      if(abs(errorT) < 1.0 ) {
        pwmT_out = 0; // Reset output if within threshold
        integralT = 0; // Reset integral term if within threshold
      }
      else{
      float dtT = (now - lastTurretTime) / 1000.0; // Time in seconds
      if (dtT == 0) dtT = 0.001; // Avoid division by zero
      integralT += errorT * dtT;
      integralT = constrain(integralT, -100, 100);
      float dErrorT = (errorT - lastErrorT) / dtT;
      // Calculate turret PWM output
      pwmT_out = Kp_turret * errorT + Ki_turret * integralT + Kd_turret * dErrorT;
      pwmT_out = constrain(pwmT_out, -maxPWM, maxPWM);
      if (abs(pwmT_out) < 400) pwmT_out = 0;
      lastErrorT = errorT;    
      lastTurretTime = now;
      }
    }
    else if ((lt < 0.1 || rt < 0.1) && (lt >= -1.5 || rt >= -1.5)) {
      pwmT_out = 0.0;
      pwmT_out = 0;
    }

    //// ======================== CONTROL END ===================////

    // Set motor speeds
    setMotor(pwmL, dirL, pwmL_out, 0);
    setMotor(pwmR, dirR, pwmR_out, 1);
    setMotor(pwmT, dirT, pwmT_out, 2);

    // Calculate rotations
    float rotL = currentTicksL / (float)TICKS_PER_WHEEL_REV;
    float rotR = currentTicksR / (float)TICKS_PER_WHEEL_REV;

    String status = String("L: ") + String(rpmL, 1) + " RPM | R: " + String(rpmR, 1) +
                " RPM | Error_Turret: " + String(errorT, 1) +
                " | PWM: L=" + String((int)pwmL_out) +
                ", R=" + String((int)pwmR_out) +
                ", T=" + String((int)pwmT_out) +
                " | Rot: L=" + String(rotL, 2) +
                ", R=" + String(rotR, 2) +
                ", T_angle=" + String(currentAngleT, 2);
    // Serial.println(status);
    // sendUDP(status);

  }
  
  // delay(100);

  /////// ================= LOCALIZATION START =====================////

  if(now- lastOdometryTime >= ODOMETRY_INTERVAL) {
    // Update odometry every ODOMETRY_INTERVAL ms
    updateOdometry(); // KF-Prediction step
    ekfYawUpdate(yaw_b, cfg);//KF - Update step updates robot_x, robot_y, robot_theta, covariance
    // updateSampledPoseFromLastDelta();
    // transmitPoseData();

    // static unsigned long lastDetailedPrint = 0;
    // if (now - lastDetailedPrint >= 1000) { // Print every 1-second
    //   Serial.println("\n PROBABILISTIC ODOM ESTIMATION:");
      printPose();
    //   printMotionModel();

    //   static unsigned long lastCovPrint = 0;
    //   if (now - lastCovPrint >= 5000) { // Print covariance every 5 seconds
    //     printCovariance();
    //     lastCovPrint = now;
    //   }

    //   float sample_x, sample_y, sample_theta;
    //   samplePose(sample_x, sample_y, sample_theta); 
    //   Serial.printf("Sampled Pose: X=%.2f, Y=%.2f, Theta=%.2f\n", sample_x, sample_y, sample_theta * 180.0 / PI);
    //   Serial.println("--------------------------------------------------");
    //   lastDetailedPrint = now;
    // }

    // if (sens.getStatus() == IMU_OK && sens.isDataValid()) {
    //     bool ekf_success = ekfYawUpdate(yaw_b, cfg);
    //     static int ekf_accept_count = 0;
    //     static int ekf_total_count = 0;
    //     static unsigned long last_ekf_stats = 0;
    //     ekf_total_count++;
    //     if (ekf_success) ekf_accept_count++;
    //     if (millis() - last_ekf_stats > 10000) { // Every 10 seconds
    //         Serial.printf("EKF Stats: %d/%d (%.1f%%) measurements accepted\n",
    //                      ekf_accept_count, ekf_total_count, 
    //                      100.0f * ekf_accept_count / ekf_total_count);
    //         bool aligned;
    //         float offset;
    //         unsigned long last_align;
    //         getEkfAlignmentInfo(aligned, offset, last_align);
    //         Serial.printf("EKF Alignment: %s, offset=%.1f°, age=%lums\n",
    //                      aligned ? "YES" : "NO", offset, millis() - last_align);
            
    //         last_ekf_stats = millis();
    //     }
    // } else {
    //     Serial.println("EKF: Using odometry only (IMU not ready)");
    // }
    // static unsigned long last_pose_tx = 0;
    // if (millis() - last_pose_tx >= 50) {
    //     transmitPoseData();
    //     last_pose_tx = millis();
    // }
    lastOdometryTime = now;
  }
}




