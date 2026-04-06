/**

 * Motor IDs:
 *   Roll  -> CAN ID 141 (0x8D)
 *   Pitch -> CAN ID 142 (0x8E)
 *
 * Protocol (from Motor_Motion_Protocol_V4.2):
 *   Send CAN ID  : 0x140 + motorID   (0x1FD roll, 0x1FE pitch)
 *   Reply CAN ID : 0x240 + motorID   (0x2FD roll, 0x2FE pitch)
 *   Frame type   : Standard, Data frame, DLC = 8 bytes
 *   Baud rate    : 1 Mbps
 *
 * Key commands used:
 *   0x9C  – Read Motor Status 2 (temp, torque current, speed, angle)
 *   0xA4  – Absolute Position Closed-Loop Control
 *   0x80  – Motor Shutdown
 *   0x81  – Motor Stop
 *
 * Build:
 *   g++ -std=c++17 -o gimbal_can gimbal_can.cpp
 *
 * Run (requires a SocketCAN interface, e.g. can0):
 *   sudo ip link set can0 up type can bitrate 1000000
 *   sudo ./gimbal_can
 */

#include <cstdint>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdexcept>
#include <string>

// ─────────────────────────────────────────────────────────
// Motor IDs and CAN ID helpers
// ─────────────────────────────────────────────────────────
static constexpr uint8_t  MOTOR_ROLL  = 141;   // 0x8D
static constexpr uint8_t  MOTOR_PITCH = 142;   // 0x8E

static constexpr uint32_t TX_BASE  = 0x140;    // send  CAN ID base
static constexpr uint32_t RX_BASE  = 0x240;    // reply CAN ID base

inline uint32_t txID(uint8_t motorID) { return TX_BASE + motorID; }
inline uint32_t rxID(uint8_t motorID) { return RX_BASE + motorID; }

// ─────────────────────────────────────────────────────────
// Command bytes (protocol §2)
// ─────────────────────────────────────────────────────────
static constexpr uint8_t CMD_READ_STATUS2   = 0x9C;
static constexpr uint8_t CMD_ABS_POSITION   = 0xA4;
static constexpr uint8_t CMD_MOTOR_SHUTDOWN = 0x80;
static constexpr uint8_t CMD_MOTOR_STOP     = 0x81;

// ─────────────────────────────────────────────────────────
// Decoded motor feedback
// ─────────────────────────────────────────────────────────
struct MotorStatus {
    int8_t   temperature_C = 0;   // 1 °C / LSB
    float    torque_A      = 0.f; // iq, 0.01 A / LSB  (int16)
    float    speed_dps     = 0.f; // 1 dps / LSB       (int16)
    float    angle_deg     = 0.f; // 1 °  / LSB        (int16, ±32767°)
};

// ─────────────────────────────────────────────────────────
// SocketCAN wrapper
// ─────────────────────────────────────────────────────────
class CANBus {
public:
    explicit CANBus(const std::string& iface) {
        sock_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (sock_ < 0)
            throw std::runtime_error("socket() failed");

        struct ifreq ifr{};
        std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
        if (ioctl(sock_, SIOCGIFINDEX, &ifr) < 0)
            throw std::runtime_error("ioctl SIOCGIFINDEX failed – is " + iface + " up?");

        struct sockaddr_can addr{};
        addr.can_family  = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(sock_, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) < 0)
            throw std::runtime_error("bind() failed");

        // 100 ms receive timeout
        struct timeval tv{ .tv_sec = 0, .tv_usec = 100'000 };
        setsockopt(sock_, SOL_SOCKET, SO_RCVTIMEO, &tv, sizeof(tv));
    }

    ~CANBus() { if (sock_ >= 0) close(sock_); }

    void send(uint32_t canID, const uint8_t data[8]) {
        struct can_frame frame{};
        frame.can_id  = canID;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);
        if (write(sock_, &frame, sizeof(frame)) != sizeof(frame))
            throw std::runtime_error("write() failed");
    }

    // Returns false on timeout / error
    bool recv(struct can_frame& out) {
        ssize_t n = read(sock_, &out, sizeof(out));
        return n == sizeof(out);
    }

private:
    int sock_ = -1;
};

// ─────────────────────────────────────────────────────────
// Motor command helpers
// ─────────────────────────────────────────────────────────
bool readStatus2(CANBus& bus, uint8_t motorID, MotorStatus& out) {
    uint8_t tx[8] = { CMD_READ_STATUS2, 0, 0, 0, 0, 0, 0, 0 };
    bus.send(txID(motorID), tx);

    struct can_frame rx{};
    if (!bus.recv(rx))             return false;
    if (rx.can_id != rxID(motorID)) return false;
    if (rx.data[0] != CMD_READ_STATUS2) return false;

    out.temperature_C = static_cast<int8_t>(rx.data[1]);

    int16_t iq    = static_cast<int16_t>(rx.data[2] | (rx.data[3] << 8));
    int16_t speed = static_cast<int16_t>(rx.data[4] | (rx.data[5] << 8));
    int16_t angle = static_cast<int16_t>(rx.data[6] | (rx.data[7] << 8));

    out.torque_A  = iq    * 0.01f;   // 0.01 A / LSB
    out.speed_dps = static_cast<float>(speed);  // 1 dps / LSB
    out.angle_deg = static_cast<float>(angle);  // 1 °   / LSB
    return true;
}

bool absolutePositionControl(CANBus& bus, uint8_t motorID,
                              float angleDeg, uint16_t maxSpeedDps,
                              MotorStatus& feedback) {
    int32_t  angleControl = static_cast<int32_t>(std::lroundf(angleDeg * 100.0f));

    uint8_t tx[8];
    tx[0] = CMD_ABS_POSITION;
    tx[1] = 0x00;
    tx[2] = static_cast<uint8_t>(maxSpeedDps);
    tx[3] = static_cast<uint8_t>(maxSpeedDps >> 8);
    tx[4] = static_cast<uint8_t>(angleControl);
    tx[5] = static_cast<uint8_t>(angleControl >> 8);
    tx[6] = static_cast<uint8_t>(angleControl >> 16);
    tx[7] = static_cast<uint8_t>(angleControl >> 24);

    bus.send(txID(motorID), tx);

    struct can_frame rx{};
    if (!bus.recv(rx))                   return false;
    if (rx.can_id != rxID(motorID))      return false;
    if (rx.data[0] != CMD_ABS_POSITION)  return false;

    feedback.temperature_C = static_cast<int8_t>(rx.data[1]);
    int16_t iq    = static_cast<int16_t>(rx.data[2] | (rx.data[3] << 8));
    int16_t speed = static_cast<int16_t>(rx.data[4] | (rx.data[5] << 8));
    int16_t angle = static_cast<int16_t>(rx.data[6] | (rx.data[7] << 8));
    feedback.torque_A  = iq    * 0.01f;
    feedback.speed_dps = static_cast<float>(speed);
    feedback.angle_deg = static_cast<float>(angle);
    return true;
}

//  (disables drive, motor goes free)
void motorShutdown(CANBus& bus, uint8_t motorID) {
    uint8_t tx[8] = { CMD_MOTOR_SHUTDOWN, 0, 0, 0, 0, 0, 0, 0 };
    bus.send(txID(motorID), tx);
}

// (holds position, keeps drive active)
void motorStop(CANBus& bus, uint8_t motorID) {
    uint8_t tx[8] = { CMD_MOTOR_STOP, 0, 0, 0, 0, 0, 0, 0 };
    bus.send(txID(motorID), tx);
}

// ─────────────────────────────────────────────────────────
// Utility: print motor status
// ─────────────────────────────────────────────────────────
void printStatus(const char* label, const MotorStatus& s) {
    printf("[%s] temp=%d°C  torque=%.2fA  speed=%.1fdps  angle=%.1f°\n",
           label, s.temperature_C, s.torque_A, s.speed_dps, s.angle_deg);
}

// ─────────────────────────────────────────────────────────
// Main – demo loop
// ─────────────────────────────────────────────────────────
int main() {
    const char* CAN_IFACE = "can0";  // change to your interface (e.g. can1)

    CANBus bus(CAN_IFACE);
    printf("CAN bus opened on %s\n", CAN_IFACE);

    // --- 1. Read initial status of both motors ---
    MotorStatus rollStatus, pitchStatus;

    if (readStatus2(bus, MOTOR_ROLL, rollStatus))
        printStatus("ROLL  status", rollStatus);
    else
        printf("[ROLL]  No reply – check wiring / ID\n");

    if (readStatus2(bus, MOTOR_PITCH, pitchStatus))
        printStatus("PITCH status", pitchStatus);
    else
        printf("[PITCH] No reply – check wiring / ID\n");

    usleep(20'000); // 20 ms

    // --- 2. Move roll to 0°, pitch to 0° (home position) ---
    printf("\nHoming both axes to 0° at 50 dps...\n");

    MotorStatus fb;
    if (absolutePositionControl(bus, MOTOR_ROLL,  0.0f, 50, fb))
        printStatus("ROLL  fb", fb);

    if (absolutePositionControl(bus, MOTOR_PITCH, 0.0f, 50, fb))
        printStatus("PITCH fb", fb);

    // Wait for motion to complete (simple open-loop delay)
    sleep(3);

    // --- 3. Move to a sample gimbal position ---
    float targetRoll_deg  =  15.0f;
    float targetPitch_deg = -10.0f;
    uint16_t maxSpeed     =  30;     // dps

    printf("\nMoving -> roll=%.1f°  pitch=%.1f°  at %u dps max\n",
           targetRoll_deg, targetPitch_deg, maxSpeed);

    if (absolutePositionControl(bus, MOTOR_ROLL,  targetRoll_deg,  maxSpeed, fb))
        printStatus("ROLL  fb", fb);

    if (absolutePositionControl(bus, MOTOR_PITCH, targetPitch_deg, maxSpeed, fb))
        printStatus("PITCH fb", fb);

    sleep(3);

    // --- 4. Telemetry read loop (10 iterations) ---
    printf("\n--- Telemetry loop ---\n");
    for (int i = 0; i < 10; ++i) {
        if (readStatus2(bus, MOTOR_ROLL,  rollStatus))  printStatus("ROLL ", rollStatus);
        if (readStatus2(bus, MOTOR_PITCH, pitchStatus)) printStatus("PITCH", pitchStatus);
        usleep(100'000); // 100 ms
    }

    // --- 5. Safe stop ---
    printf("\nStopping motors...\n");
    motorStop(bus, MOTOR_ROLL);
    motorStop(bus, MOTOR_PITCH);

    printf("Done.\n");
    return 0;
}