#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <unistd.h>
#include <net/if.h>
#include <sys/socket.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdexcept>
#include <string>
#include <time.h>

// ═════════════════════════════════════════════════════════
// CONTROL MODE — change this one line to switch modes
// ═════════════════════════════════════════════════════════
enum class ControlMode { Position, Torque };
static constexpr ControlMode CONTROL_MODE = ControlMode::Position;

// ── Config ────────────────────────────────────────────────
static constexpr uint8_t  MOTOR_ROLL  = 1;
static constexpr uint8_t  MOTOR_PITCH = 2;
static constexpr uint32_t TX_BASE     = 0x140;
static constexpr uint32_t RX_BASE     = 0x240;

// Commands
static constexpr uint8_t CMD_READ_STATUS2   = 0x9C;
static constexpr uint8_t CMD_ABS_POSITION   = 0xA4;
static constexpr uint8_t CMD_TORQUE         = 0xA1;
static constexpr uint8_t CMD_BRAKE_OPEN     = 0x77;
static constexpr uint8_t CMD_BRAKE_LOCK     = 0x78;
static constexpr uint8_t CMD_MOTOR_STOP     = 0x81;
static constexpr uint8_t CMD_MOTOR_SHUTDOWN = 0x80;

// Position mode settle thresholds
static constexpr float    SETTLE_SPEED_DPS   = 3.0f;
static constexpr float    SETTLE_ANGLE_DEG   = 1.0f;
static constexpr uint32_t RESEND_INTERVAL_MS = 500;

// Torque mode safety clamp — absolute max amps sent to either motor
// Raise only after you are confident the mechanics are clear
static constexpr float TORQUE_MAX_AMPS = 2.0f;

static uint32_t millis() {
    struct timespec ts{};
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (uint32_t)(ts.tv_sec * 1000ULL + ts.tv_nsec / 1000000ULL);
}

static void sleepMs(uint32_t ms) { usleep(ms * 1000); }

// ── Motor status ──────────────────────────────────────────
struct MotorStatus {
    int8_t temperature_C = 0;
    float  torque_A  = 0.f;
    float  speed_dps = 0.f;
    float  angle_deg = 0.f;
    bool   valid     = false;
};

// ── CAN bus ───────────────────────────────────────────────
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

        printf("SocketCAN interface %s open at 1 Mbit/s\n", iface.c_str());
    }

    ~CANBus() { if (sock_ >= 0) close(sock_); }

    void send(uint32_t id, const uint8_t data[8]) {
        struct can_frame frame{};
        frame.can_id  = id;
        frame.can_dlc = 8;
        std::memcpy(frame.data, data, 8);
        if (write(sock_, &frame, sizeof(frame)) != sizeof(frame))
            throw std::runtime_error("write() failed");
    }

    bool recvLatest(uint32_t wantID, struct can_frame& out, uint32_t timeoutMs = 25) {
        bool got = false;
        uint32_t deadline = millis() + timeoutMs;

        while (millis() < deadline) {
            fd_set rfds;
            FD_ZERO(&rfds);
            FD_SET(sock_, &rfds);

            uint32_t rem = deadline - millis();
            struct timeval tv{ .tv_sec = 0, .tv_usec = (long)(rem * 1000) };

            int ret = select(sock_ + 1, &rfds, nullptr, nullptr, &tv);
            if (ret > 0) {
                struct can_frame m{};
                if (read(sock_, &m, sizeof(m)) == (ssize_t)sizeof(m)) {
                    if (m.can_id == wantID) { out = m; got = true; }
                }
                continue;
            }
            break; 
        }
        return got;
    }

private:
    int sock_ = -1;
};

// ── Decode reply ──────────────────────────────────────────
static MotorStatus decode(const struct can_frame& m) {
    MotorStatus s;
    s.temperature_C = (int8_t)m.data[1];
    s.torque_A  = (int16_t)(m.data[2] | m.data[3] << 8) * 0.01f;
    s.speed_dps = (float)(int16_t)(m.data[4] | m.data[5] << 8);
    s.angle_deg = (float)(int16_t)(m.data[6] | m.data[7] << 8);
    s.valid     = true;
    return s;
}

// ── Low-level motor commands ──────────────────────────────
MotorStatus readStatus(CANBus& bus, uint8_t id) {
    uint8_t tx[8] = { CMD_READ_STATUS2, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, tx);
    struct can_frame rx{};
    if (!bus.recvLatest(RX_BASE + id, rx, 30)) return {};
    if (rx.data[0] != CMD_READ_STATUS2)         return {};
    return decode(rx);
}

void sendAbsPosition(CANBus& bus, uint8_t id, float deg, uint16_t maxDps) {
    int32_t ac = (int32_t)std::lroundf(deg * 100.f);
    uint8_t tx[8] = {
        CMD_ABS_POSITION, 0x00,
        (uint8_t)maxDps,    (uint8_t)(maxDps >> 8),
        (uint8_t)ac,        (uint8_t)(ac >>  8),
        (uint8_t)(ac >> 16),(uint8_t)(ac >> 24)
    };
    bus.send(TX_BASE + id, tx);
    struct can_frame rx{}; bus.recvLatest(RX_BASE + id, rx, 20); // drain echo
}

// torqueA is clamped to ±TORQUE_MAX_AMPS before sending
MotorStatus sendTorque(CANBus& bus, uint8_t id, float torqueA) {
    // Safety clamp
    if (torqueA >  TORQUE_MAX_AMPS) torqueA =  TORQUE_MAX_AMPS;
    if (torqueA < -TORQUE_MAX_AMPS) torqueA = -TORQUE_MAX_AMPS;

    int16_t iq = (int16_t)std::lroundf(torqueA * 100.f); // 0.01 A/LSB
    uint8_t tx[8] = {
        CMD_TORQUE, 0x00, 0x00, 0x00,
        (uint8_t)iq, (uint8_t)(iq >> 8),
        0x00, 0x00
    };
    bus.send(TX_BASE + id, tx);
    struct can_frame rx{};
    if (!bus.recvLatest(RX_BASE + id, rx, 30)) return {};
    if (rx.data[0] != CMD_TORQUE)               return {};
    return decode(rx);
}

void brakeRelease(CANBus& bus, uint8_t id) {
    uint8_t t[8] = { CMD_BRAKE_OPEN, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, t);
    struct can_frame rx{}; bus.recvLatest(RX_BASE + id, rx, 30);
    sleepMs(50); // give brake time to physically open
}

void brakeLock(CANBus& bus, uint8_t id) {
    uint8_t t[8] = { CMD_BRAKE_LOCK, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, t);
    struct can_frame rx{}; bus.recvLatest(RX_BASE + id, rx, 30);
}

void motorStop    (CANBus& bus, uint8_t id) { uint8_t t[8]={CMD_MOTOR_STOP,    0,0,0,0,0,0,0}; bus.send(TX_BASE+id,t); }
void motorShutdown(CANBus& bus, uint8_t id) { uint8_t t[8]={CMD_MOTOR_SHUTDOWN,0,0,0,0,0,0,0}; bus.send(TX_BASE+id,t); }

// ── Print helpers ─────────────────────────────────────────
void printStatus(const char* lbl, const MotorStatus& s) {
    if (!s.valid) { printf("[%-5s] no reply\n", lbl); return; }
    printf("[%-5s] %3d C  torque=%+6.2f A  speed=%+7.1f dps  angle=%+8.1f deg\n",
           lbl, s.temperature_C, s.torque_A, s.speed_dps, s.angle_deg);
}

void printAllStatus(CANBus& bus) {
    printStatus("ROLL ", readStatus(bus, MOTOR_ROLL));
    printStatus("PITCH", readStatus(bus, MOTOR_PITCH));
}

// ── Position mode: move both axes and wait for settle ─────
void moveBothPosition(CANBus& bus,
                      float rollDeg,  uint16_t rollSpd,
                      float pitchDeg, uint16_t pitchSpd) {
    printf("  -> roll=%.1f deg @ %u dps  |  pitch=%.1f deg @ %u dps\n",
           rollDeg, rollSpd, pitchDeg, pitchSpd);

    sendAbsPosition(bus, MOTOR_ROLL,  rollDeg,  rollSpd);
    sleepMs(2);
    sendAbsPosition(bus, MOTOR_PITCH, pitchDeg, pitchSpd);

    uint32_t lastResend = millis();

    while (true) {
        if (millis() - lastResend >= RESEND_INTERVAL_MS) {
            sendAbsPosition(bus, MOTOR_ROLL,  rollDeg,  rollSpd);
            sleepMs(2);
            sendAbsPosition(bus, MOTOR_PITCH, pitchDeg, pitchSpd);
            lastResend = millis();
        }

        MotorStatus r = readStatus(bus, MOTOR_ROLL);
        MotorStatus p = readStatus(bus, MOTOR_PITCH);
        printStatus("ROLL ", r);
        printStatus("PITCH", p);

        bool rollDone  = r.valid && fabsf(r.angle_deg - rollDeg)  < SETTLE_ANGLE_DEG
                                 && fabsf(r.speed_dps)            < SETTLE_SPEED_DPS;
        bool pitchDone = p.valid && fabsf(p.angle_deg - pitchDeg) < SETTLE_ANGLE_DEG
                                 && fabsf(p.speed_dps)            < SETTLE_SPEED_DPS;

        if (rollDone && pitchDone) { printf("  -> settled\n"); return; }
        sleepMs(50);
    }
}

// ── Torque mode: stream constant torque for durationMs ────
// Sends torque commands at ~50 Hz, printing feedback each cycle.
// Zeroes torque and locks brake when done.
void runBothTorque(CANBus& bus,
                   float rollA, float pitchA,
                   uint32_t durationMs = 2000) {
    printf("  -> roll=%.3f A  |  pitch=%.3f A  for %u ms\n",
           rollA, pitchA, durationMs);
    printf("  -> clamped to +/-%.1f A max\n", TORQUE_MAX_AMPS);

    // Release brakes — required before 0xA1 per manual §2.26
    brakeRelease(bus, MOTOR_ROLL);
    brakeRelease(bus, MOTOR_PITCH);
    printf("  -> brakes released\n");

    uint32_t deadline = millis() + durationMs;
    while (millis() < deadline) {
        MotorStatus r = sendTorque(bus, MOTOR_ROLL,  rollA);
        MotorStatus p = sendTorque(bus, MOTOR_PITCH, pitchA);
        printStatus("ROLL ", r);
        printStatus("PITCH", p);
        sleepMs(20); // ~50 Hz
    }

    // Always zero torque first, then lock brake
    sendTorque(bus, MOTOR_ROLL,  0.f);
    sendTorque(bus, MOTOR_PITCH, 0.f);
    sleepMs(20);
    brakeLock(bus, MOTOR_ROLL);
    brakeLock(bus, MOTOR_PITCH);
    printf("  -> torque zeroed, brakes locked\n");
}

// ── CLI ───────────────────────────────────────────────────
void printUsage(const char* exe) {
    const char* modeStr = (CONTROL_MODE == ControlMode::Position) ? "Position" : "Torque";
    printf("Active mode: %s\n\n", modeStr);
    printf("Usage:\n");
    printf("  %s status\n", exe);
    printf("  %s stop\n", exe);

    if (CONTROL_MODE == ControlMode::Position) {
        printf("  %s home [speed_dps]\n", exe);
        printf("  %s move <roll_deg> <pitch_deg> [speed_dps]\n", exe);
        printf("\nExamples:\n");
        printf("  %s home\n", exe);
        printf("  %s move 0 90 30\n", exe);
    } else {
        printf("  %s move <roll_A> <pitch_A> [duration_ms]\n", exe);
        printf("\nExamples:\n");
        printf("  %s move 0.3 0.3 2000\n", exe);
        printf("  %s move -0.3 0.3 1000\n", exe);
        printf("\nNote: values are Amps. Max is +/-%.1f A (TORQUE_MAX_AMPS).\n", TORQUE_MAX_AMPS);
        printf("      home command not available in torque mode.\n");
    }
}

float parseFloat(const char* text, const char* name) {
    char* end = nullptr;
    float v = std::strtof(text, &end);
    if (end == text || (end && *end != '\0'))
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    return v;
}

uint32_t parseUInt(const char* text, const char* name) {
    char* end = nullptr;
    unsigned long v = std::strtoul(text, &end, 10);
    if (end == text || (end && *end != '\0'))
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    return (uint32_t)v;
}

// ── Main ──────────────────────────────────────────────────
int main(int argc, char** argv) {
    try {
        if (argc < 2) { printUsage(argv[0]); return 1; }

        const char* CAN_IFACE = "can0"; 
        CANBus bus(CAN_IFACE);

        const std::string cmd = argv[1];

        // ── Commands available in both modes ──────────────
        if (cmd == "status") {
            printAllStatus(bus);
            return 0;
        }

        if (cmd == "stop") {
            printf("\n--- Stop ---\n");
            sendTorque(bus, MOTOR_ROLL,  0.f);
            sendTorque(bus, MOTOR_PITCH, 0.f);
            sleepMs(20);
            motorStop(bus, MOTOR_ROLL);
            motorStop(bus, MOTOR_PITCH);
            printf("Done.\n");
            return 0;
        }

        // ── Position mode commands ────────────────────────
        if (CONTROL_MODE == ControlMode::Position) {

            if (cmd == "home") {
                uint16_t spd = argc >= 3 ? (uint16_t)parseUInt(argv[2], "speed_dps") : 30;
                printf("\n--- Home ---\n");
                moveBothPosition(bus, 0.f, spd, 0.f, spd);
                printf("\n--- Final status ---\n");
                printAllStatus(bus);

            } else if (cmd == "move") {
                if (argc < 4) { printUsage(argv[0]); return 1; }
                float    roll  = parseFloat(argv[2], "roll_deg");
                float    pitch = parseFloat(argv[3], "pitch_deg");
                uint16_t spd   = argc >= 5 ? (uint16_t)parseUInt(argv[4], "speed_dps") : 30;
                printf("\n--- Move ---\n");
                moveBothPosition(bus, roll, spd, pitch, spd);
                printf("\n--- Final status ---\n");
                printAllStatus(bus);

            } else {
                printUsage(argv[0]); return 1;
            }

        // ── Torque mode commands ──────────────────────────
        } else {

            if (cmd == "home") {
                printf("ERROR: home is not available in torque mode.\n");
                printf("       Switch CONTROL_MODE to Position to use home.\n");
                return 1;

            } else if (cmd == "move") {
                if (argc < 4) { printUsage(argv[0]); return 1; }
                float    rollA  = parseFloat(argv[2], "roll_A");
                float    pitchA = parseFloat(argv[3], "pitch_A");
                uint32_t dur    = argc >= 5 ? parseUInt(argv[4], "duration_ms") : 2000;
                printf("\n--- Torque ---\n");
                runBothTorque(bus, rollA, pitchA, dur);
                printf("\n--- Final status ---\n");
                printAllStatus(bus);

            } else {
                printUsage(argv[0]); return 1;
            }
        }

    } catch (const std::exception& e) {
        fprintf(stderr, "ERROR: %s\n", e.what());
        return 1;
    }
    return 0;
}
