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

// void setup() {
//   pinMode(LED_BUILTIN, OUTPUT);
//   Serial.begin(115200);
//   Serial.println("Setup complete. LED will blink every second.");
// }

// void loop() {
//   digitalWrite(LED_BUILTIN, HIGH);
//   delay(1000);
//   digitalWrite(LED_BUILTIN, LOW);
//   delay(1000);
// }

// // // float lx, ly, rx, ry, lt, rt;
int a, b, x, y;

const char* webpage = R"rawliteral(
<!DOCTYPE html>
<html>
<head>
  <meta name="viewport" content="width=device-width, initial-scale=1">
  <title>HAMR</title>
  <style>
    body {
      font-family: 'Segoe UI', Tahoma, Geneva, Verdana, sans-serif;
      text-align: center;
      background: linear-gradient(135deg, #32459bff 0%, #764ba2 100%);
      margin: 0;
      padding: 20px;
      color: white;
      min-height: 100vh;
    }

    .container {
      max-width: 500px;
      margin: 0 auto;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 20px;
      padding: 30px;
      backdrop-filter: blur(10px);
      box-shadow: 0 8px 32px rgba(0, 0, 0, 0.3);
      border: 1px solid rgba(255, 255, 255, 0.2);
    }

    h2 {
      margin-top: 0;
      margin-bottom: 30px;
      font-size: 2.2em;
      text-shadow: 2px 2px 4px rgba(0, 0, 0, 0.3);
      display: flex;
      align-items: center;
      justify-content: center;
      gap: 15px;
    }

    #joystickZone {
      width: 200px;
      height: 200px;
      margin: 20px auto;
      background: linear-gradient(145deg, #e0e0e0, #c0c0c0);
      border-radius: 50%;
      position: relative;
      touch-action: none;
      box-shadow: inset 8px 8px 16px rgba(0, 0, 0, 0.2),
                  inset -8px -8px 16px rgba(255, 255, 255, 0.7);
    }

    #stick {
      width: 50px;
      height: 50px;
      background: linear-gradient(145deg, #555, #333);
      border-radius: 50%;
      position: absolute;
      top: 75px;
      left: 75px;
      cursor: pointer;
      transition: all 0.1s ease;
      box-shadow: 4px 4px 8px rgba(0, 0, 0, 0.3),
                  -2px -2px 4px rgba(255, 255, 255, 0.1);
    }

    #stick:hover {
      transform: scale(1.1);
    }

    .control-group {
      margin: 25px 0;
      padding: 20px;
      background: rgba(255, 255, 255, 0.05);
      border-radius: 15px;
      border: 1px solid rgba(255, 255, 255, 0.1);
    }

    .control-group h3 {
      margin: 0 0 15px 0;
      font-size: 1.3em;
      color: #fff;
    }

    .slider-container {
      display: flex;
      align-items: center;
      gap: 15px;
      justify-content: center;
      margin: 15px 0;
    }

    .slider {
      -webkit-appearance: none;
      width: 200px;
      height: 8px;
      border-radius: 4px;
      background: linear-gradient(90deg, #ff6b6b 0%, #4ecdc4 50%, #45b7d1 100%);
      outline: none;
      position: relative;
    }

    .slider::-webkit-slider-thumb {
      -webkit-appearance: none;
      appearance: none;
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
      transition: all 0.2s ease;
    }

    .slider::-webkit-slider-thumb:hover {
      transform: scale(1.2);
      box-shadow: 0 6px 12px rgba(0, 0, 0, 0.4);
    }

    .slider::-moz-range-thumb {
      width: 24px;
      height: 24px;
      border-radius: 50%;
      background: linear-gradient(145deg, #fff, #ddd);
      cursor: pointer;
      border: none;
      box-shadow: 0 4px 8px rgba(0, 0, 0, 0.3);
    }

    .slider-labels {
      display: flex;
      justify-content: space-between;
      width: 200px;
      margin: 10px auto 0;
      font-size: 0.9em;
      color: rgba(255, 255, 255, 0.8);
    }

    .slider-value {
      min-width: 60px;
      padding: 8px 12px;
      background: rgba(255, 255, 255, 0.1);
      border-radius: 8px;
      font-family: monospace;
      font-size: 1.1em;
      border: 1px solid rgba(255, 255, 255, 0.2);
      margin-top: 10px;
    }

    button {
      padding: 12px 24px;
      font-size: 16px;
      margin: 10px;
      background: linear-gradient(145deg, #4ecdc4, #44a08d);
      color: white;
      border: none;
      border-radius: 25px;
      cursor: pointer;
      transition: all 0.3s ease;
      font-weight: 600;
      box-shadow: 0 4px 15px rgba(68, 160, 141, 0.3);
    }

    button:hover {
      transform: translateY(-2px);
      box-shadow: 0 6px 20px rgba(68, 160, 141, 0.4);
      background: linear-gradient(145deg, #5dd4cc, #4ecdc4);
    }

    button:active {
      transform: translateY(0);
    }

    input[type="number"] {
      width: 100px;
      padding: 12px;
      font-size: 16px;
      margin-right: 10px;
      border: 2px solid rgba(255, 255, 255, 0.2);
      border-radius: 12px;
      background: rgba(255, 255, 255, 0.1);
      color: white;
      text-align: center;
    }

    input[type="number"]::placeholder {
      color: rgba(255, 255, 255, 0.6);
    }

    input[type="number"]:focus {
      outline: none;
      border-color: #4ecdc4;
      box-shadow: 0 0 10px rgba(78, 205, 196, 0.3);
    }

    .status-indicator {
      display: inline-block;
      width: 12px;
      height: 12px;
      border-radius: 50%;
      background: #4ecdc4;
      animation: pulse 2s infinite;
    }

    @keyframes pulse {
      0% { opacity: 1; }
      50% { opacity: 0.5; }
      100% { opacity: 1; }
    }

    .info-panel {
      margin-top: 20px;
      padding: 15px;
      background: rgba(0, 0, 0, 0.2);
      border-radius: 10px;
      font-family: monospace;
      font-size: 0.9em;
      text-align: left;
    }

    .trigger-control {
      width: 100%;
      max-width: 300px;
      margin: 0 auto;
    }

    .trigger-end-label {
      font-size: 1.1em;
      font-weight: 600;
      color: rgba(255, 255, 255, 0.9);
      min-width: 30px;
    }
  </style>
</head>
<body>
  <div class="container">
    <h2>HAMR Robot Control <span class="status-indicator"></span></h2>

    <div class="control-group">
      <h3>Drive Control</h3>
      <div id="joystickZone">
        <div id="stick"></div>
      </div>
    </div>

    <div class="control-group">
      <h3>Turret Control</h3>
      <div class="trigger-control">
        <div class="slider-container">
          <span class="trigger-end-label">LT</span>
          <input type="range" id="turretSlider" class="slider" min="-100" max="100" value="0" step="1">
          <span class="trigger-end-label">RT</span>
        </div>
        <div class="slider-labels">
          <span>-100%</span>
          <span>0%</span>
          <span>+100%</span>
        </div>
        <div class="slider-value" id="turretValue">0%</div>
      </div>
    </div>

    <div class="control-group">
      <h3>Turret Position</h3>
      <input type="number" id="angleInput" placeholder="Angle (0-360)" min="0" max="360" step="1">
      <button onclick="sendTurretAngle()">Set Position</button>
    </div>

    <div class="control-group">
      <button onclick="resetOdometry()">Reset Odometry</button>
      <button onclick="getPose()">Get Pose</button>
    </div>

    <div class="info-panel" id="infoPanel">
      Robot Status: Ready<br>
      Position: X=0.000, Y=0.000, θ=0.0°<br>
      LT: 0.00 | RT: 0.00<br>
      Last Command: None
    </div>
  </div>

  <script>
    const stick = document.getElementById('stick');
    const zone = document.getElementById('joystickZone');
    const turretSlider = document.getElementById('turretSlider');
    const turretValue = document.getElementById('turretValue');
    const infoPanel = document.getElementById('infoPanel');
    
    let dragging = false;
    let currentTurretValue = 0;

    // Joystick event listeners
    zone.addEventListener('touchstart', startDrag);
    zone.addEventListener('touchmove', drag);
    zone.addEventListener('touchend', endDrag);
    zone.addEventListener('mousedown', startDrag);
    zone.addEventListener('mousemove', drag);
    zone.addEventListener('mouseup', endDrag);
    zone.addEventListener('mouseleave', endDrag);

    // Turret slider event listeners
    turretSlider.addEventListener('input', function() {
      currentTurretValue = parseInt(this.value);
      turretValue.textContent = currentTurretValue + '%';
      
      if (currentTurretValue < 0) {
        // Left side = LT trigger
        const ltValue = Math.abs(currentTurretValue) / 100.0;
        sendTrigger('lt', ltValue);
      } else if (currentTurretValue > 0) {
        // Right side = RT trigger  
        const rtValue = currentTurretValue / 100.0;
        sendTrigger('rt', rtValue);
      } else {
        // Center = stop
        sendTrigger('stop', 0);
      }
      
      updateInfoPanel();
    });

    // Auto-return to center when released
    turretSlider.addEventListener('mouseup', returnToCenter);
    turretSlider.addEventListener('touchend', returnToCenter);

    function returnToCenter() {
      setTimeout(() => {
        if (!turretSlider.matches(':active')) {
          turretSlider.value = 0;
          currentTurretValue = 0;
          turretValue.textContent = '0%';
          sendTrigger('stop', 0);
          updateInfoPanel();
        }
      }, 100);
    }

    function startDrag(e) {
      dragging = true;
      e.preventDefault();
    }

    function drag(e) {
      if (!dragging) return;
      e.preventDefault();
      
      let x = (e.touches ? e.touches[0].clientX : e.clientX) - zone.getBoundingClientRect().left;
      let y = (e.touches ? e.touches[0].clientY : e.clientY) - zone.getBoundingClientRect().top;
      
      let dx = x - 100;
      let dy = y - 100;
      
      let dist = Math.min(Math.sqrt(dx * dx + dy * dy), 75);
      let angle = Math.atan2(dy, dx);
      
      let stickX = Math.cos(angle) * dist + 75;
      let stickY = Math.sin(angle) * dist + 75;
      
      stick.style.left = stickX + 'px';
      stick.style.top = stickY + 'px';
      
      let normX = ((stickX - 75) / 75).toFixed(2);
      let normY = ((stickY - 75) / 75).toFixed(2);
      
      fetch(`/move?x=${normX}&y=${normY}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Drive: X=${normX}, Y=${normY}`);
        })
        .catch(err => console.error('Drive command failed:', err));
    }

    function endDrag() {
      dragging = false;
      stick.style.left = '75px';
      stick.style.top = '75px';
      fetch(`/move?x=0&y=0`)
        .then(() => updateInfoPanel('Drive: Stopped'))
        .catch(err => console.error('Stop command failed:', err));
    }

    function sendTrigger(trigger, value) {
      if (trigger === 'stop') {
        fetch(`/trigger?btn=stop`)
          .then(response => response.text())
          .then(data => {
            console.log('Turret stopped');
          })
          .catch(err => console.error('Stop command failed:', err));
      } else {
        fetch(`/trigger?btn=${trigger}&value=${value}`)
          .then(response => response.text())
          .then(data => {
            console.log(`${trigger.toUpperCase()} set to ${value.toFixed(2)}`);
          })
          .catch(err => console.error(`${trigger} command failed:`, err));
      }
    }

    function sendTurretAngle() {
      const angle = document.getElementById('angleInput').value;
      if (angle === '') {
        alert('Please enter an angle value');
        return;
      }
      
      fetch(`/setTurretAngle?angle=${angle}`)
        .then(response => response.text())
        .then(data => {
          updateInfoPanel(`Turret angle set to ${angle}°`);
        })
        .catch(err => {
          console.error('Turret angle command failed:', err);
          updateInfoPanel('Turret angle command failed');
        });
    }

    function resetOdometry() {
      fetch('/reset')
        .then(response => response.text())
        .then(data => {
          updateInfoPanel('Odometry reset successfully');
          getPose();
        })
        .catch(err => {
          console.error('Reset failed:', err);
          updateInfoPanel('Reset command failed');
        });
    }

    function getPose() {
      fetch('/pose')
        .then(response => response.json())
        .then(data => {
          updateInfoPanel(`Position: X=${data.x.toFixed(3)}, Y=${data.y.toFixed(3)}, θ=${(data.theta * 180 / Math.PI).toFixed(1)}°`);
        })
        .catch(err => {
          console.error('Get pose failed:', err);
          updateInfoPanel('Failed to get pose data');
        });
    }

    function updateInfoPanel(message) {
      const timestamp = new Date().toLocaleTimeString();
      let triggerStatus = 'None';
      
      if (currentTurretValue < 0) {
        triggerStatus = `LT: ${(Math.abs(currentTurretValue) / 100).toFixed(2)}`;
      } else if (currentTurretValue > 0) {
        triggerStatus = `RT: ${(currentTurretValue / 100).toFixed(2)}`;
      }
      
      infoPanel.innerHTML = `
        Robot Status: Active<br>
        Turret Control: ${triggerStatus}<br>
        Last Command: ${message || 'None'}<br>
        Time: ${timestamp}
      `;
    }

    // Initialize display
    window.addEventListener('load', function() {
      turretSlider.value = 0;
      turretValue.textContent = '0%';
      updateInfoPanel('System initialized');
    });

    // Auto-update pose every 2 seconds
    setInterval(getPose, 2000);
  </script>
</body>
</html>
)rawliteral";

WebServer server(80);

// WIFI CREDENTIALS
const char* ssid = "HAMR";
const char* password = "123571113";

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

// IMU pins
const int IMU_SDA = 38; 
const int IMU_SCL = 39;

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
const unsigned long PID_INTERVAL = 50;
static unsigned long lastUdpTime = 0;

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
// Latest commands received over UART (ROS)
volatile float uart_left_cmd = 0.0f;
volatile float uart_right_cmd = 0.0f;
volatile float uart_turret_cmd = 0.0f;
volatile uint32_t last_uart_cmd_ms = 0;

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

#pragma pack(pop)

static const size_t CMD_SIZE  = sizeof(CmdPacket);   // 2-float
static const size_t CMD3_SIZE = sizeof(Cmd3Packet);  // 3-float
static const size_t ENC_SIZE  = sizeof(EncPacket);

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

void setup() {

  Serial.begin(115200);
  Serial.println("ESP32 Ready");
  initOdometry(); // Initialize odometry
  WiFi.softAP(ssid, password, 5, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  server.on("/", HTTP_GET, []() {
    server.send(200, "text/html", webpage);  // Serve the HTML page
  });
  // server.on("/move", HTTP_GET, []() {
  //   String xVal = server.arg("x");
  //   String yVal = server.arg("y");
  //   joyX= xVal.toFloat();
  //   joyY= -yVal.toFloat();
  //   Serial.printf("Joystick X: %.2f, Y: %.2f\n", joyX, joyY);
  //   server.send(200, "text/plain", "OK");
  // });
  // Joystick movement (single virtual joystick)
server.on("/move", HTTP_GET, []() {
  float x = server.arg("x").toFloat();
  float y = server.arg("y").toFloat();
  joyX = x;
  joyY = -y;
  server.send(200, "text/plain", "Movement received");
});

// Trigger buttons: LT or RT
server.on("/trigger", HTTP_GET, []() {
  String btn = server.arg("btn");
  float value = server.arg("value").toFloat();  // Use 'value' param

  if (btn == "lt") {
    pwmT_out = -value * maxPWM;
  } else if (btn == "rt") {
    pwmT_out = value * maxPWM;
  } else if (btn == "stop") {
    pwmT_out = 0;
  }

  setMotor(pwmT, dirT, pwmT_out, 2);
  server.send(200, "text/plain", "Trigger received: " + btn);
});

// Turret angle input
server.on("/setTurretAngle", HTTP_GET, []() {
  float angle = server.arg("angle").toFloat();
  inputTurretAngle = angle;
  targetTurretAngle = inputTurretAngle * turretGearRatio; // Apply gear ratio
  server.send(200, "text/plain", "Turret angle set.");
});

  server.onNotFound([]() {
    server.send(404, "text/plain", "404 Not Found");
  });
}

void setup() {

  Serial.begin(115200);
  Serial.println("ESP32 Ready");

  initOdometry(); // Initialize odometry
  if (ekf.initIMU(IMU_SDA, IMU_SCL)) {
    ekf.initEKF();
    Serial.println("EKF and IMU initialized successfully");
  } else {
    Serial.println("Warning: IMU initialization failed, using odometry only");
    use_ekf = false;
  }

  WiFi.softAP(ssid, password, 5, 0, 2);
  IPAddress myIP = WiFi.softAPIP();
  Serial.print("ESP IP: ");
  Serial.println(myIP);

  setupWebServerEndpoints(); // Setup web server endpoints
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
  //Serial.println("Commands: f=forward, b=backward, r=right, l=left, s=stop, +=faster, -=slower");
  Serial.println("Send joystick data like: LX:0.00 LY:0.00 RX:0.00 RY:0.00 LT:0.00 RT:0.00 A:0 B:0 X:0 Y:0");
  Serial.printf("Initial Speed PWM: %.0f\n", basePWM);
}

void loop() {
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
      uart_left_cmd   = cmd3.left;
      uart_right_cmd  = cmd3.right;
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

  // Handle serial commands
  //   Read joystick data from serial
  //   if (Serial.available()) {
  //   String msg = Serial.readStringUntil('\n');}

    //-----------------------TELEOP PROTOCOL------------------------------
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

    // Calculate ticks and send to Micro ROS Bridge to Publish 
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

    // Joystick-based differential drive control:
    // Negative ly because joystick up may be negative, adjust if needed
    // Normalize joystick values to -1 to 1 range
    // float forward = ly;  
    // float turn = rx;

    // // HTTML Joystick control
    // float forward = joyY;  // Forward/backward control from joystick
    // float turn = joyX;     // Left/right control from joystick

    bool useUdp = (millis() - lastUdpTime < 100);
    bool useUart = (millis() - last_uart_cmd_ms < 150);

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
    else{
    float forward = useUdp ? ly : joyY;
    float turn = useUdp ? rx : joyX;

    forward *= 0.8f;
    turn    *= 0.8f;

    // Combine for left and right motor base PWM
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

    // Turret control based on HTML joystick input
    // float turretSpeed = joyturretX * maxPWM;  // or joyturretY * maxPWM;
    // pwmT_out = turretSpeed;
    // noInterrupts();
    // long currentTicksT = ticksT;
    // interrupts();
    // currentAngleT = currentTicksT * DEGREES_PER_TURRET_TICK;
    // currentAngleT = fmod(currentAngleT / turretGearRatio, 360.0);
    // setMotor(pwmT, dirT, turretSpeed, 2);

    if (lt > 0.1 || rt > 0.1){
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
    Serial.println(status);
    // sendUDP(status);

  }

  /////// ================= LOCALIZATION START =====================////

  if(now- lastOdometryTime >= ODOMETRY_INTERVAL) {
    // Update odometry every ODOMETRY_INTERVAL ms
    updateOdometry();

    static unsigned long lastDetailedPrint = 0;
    if (now - lastDetailedPrint >= 2000) { // Print every 2-second
      Serial.println("\n PROBABILISTIC ODOM ESTIMATION:");
      printPose();
      printMotionModel();

      static unsigned long lastCovPrint = 0;
      if (now - lastCovPrint >= 5000) { // Print covariance every 5 seconds
        // printCovariance();
        lastCovPrint = now;
      }

      float sample_x, sample_y, sample_theta;
      samplePose(sample_x, sample_y, sample_theta); 
      // Serial.printf("Sampled Pose: X=%.2f, Y=%.2f, Theta=%.2f\n", sample_x, sample_y, sample_theta * 180.0 / PI);
      // Serial.println("--------------------------------------------------");
      lastDetailedPrint = now;
    }
    lastOdometryTime = now;
  }
}


