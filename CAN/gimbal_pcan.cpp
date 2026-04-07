#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <string>
#include <windows.h>


enum class ControlMode { Position, Torque };
static constexpr ControlMode CONTROL_MODE = ControlMode::Position;


typedef WORD  TPCANHandle;
typedef BYTE  TPCANType;
typedef DWORD TPCANBaudrate;
typedef DWORD TPCANStatus;
typedef BYTE  TPCANMessageType;

static constexpr TPCANHandle      PCAN_USBBUS1          = 0x51;
static constexpr TPCANBaudrate    PCAN_BAUD_1M          = 0x0014;
static constexpr TPCANStatus      PCAN_ERROR_OK         = 0x00000;
static constexpr TPCANStatus      PCAN_ERROR_QRCVEMPTY  = 0x00020;
static constexpr TPCANMessageType PCAN_MESSAGE_STANDARD = 0x00;

#pragma pack(push, 1)
struct TPCANMsg       { DWORD ID; TPCANMessageType MSGTYPE; BYTE LEN; BYTE DATA[8]; };
struct TPCANTimestamp { DWORD millis; WORD millis_overflow; WORD micros; };
#pragma pack(pop)

// ── Runtime DLL loader ────────────────────────────────────
typedef TPCANStatus (__stdcall *pfn_CAN_Initialize)  (TPCANHandle, TPCANBaudrate, TPCANType, DWORD, WORD);
typedef TPCANStatus (__stdcall *pfn_CAN_Uninitialize)(TPCANHandle);
typedef TPCANStatus (__stdcall *pfn_CAN_Read)        (TPCANHandle, TPCANMsg*, TPCANTimestamp*);
typedef TPCANStatus (__stdcall *pfn_CAN_Write)       (TPCANHandle, TPCANMsg*);
typedef TPCANStatus (__stdcall *pfn_CAN_GetErrorText)(TPCANStatus, WORD, LPSTR);

static pfn_CAN_Initialize   g_CAN_Initialize   = nullptr;
static pfn_CAN_Uninitialize g_CAN_Uninitialize = nullptr;
static pfn_CAN_Read         g_CAN_Read         = nullptr;
static pfn_CAN_Write        g_CAN_Write        = nullptr;
static pfn_CAN_GetErrorText g_CAN_GetErrorText = nullptr;

static void loadPCANDll() {
    HMODULE dll = LoadLibraryA("PCANBasic.dll");
    if (!dll) throw std::runtime_error("Cannot load PCANBasic.dll");
    auto load = [&](const char* n) -> FARPROC {
        FARPROC p = GetProcAddress(dll, n);
        if (!p) throw std::runtime_error(std::string("GetProcAddress: ") + n);
        return p;
    };
    g_CAN_Initialize   = (pfn_CAN_Initialize)  load("CAN_Initialize");
    g_CAN_Uninitialize = (pfn_CAN_Uninitialize)load("CAN_Uninitialize");
    g_CAN_Read         = (pfn_CAN_Read)        load("CAN_Read");
    g_CAN_Write        = (pfn_CAN_Write)       load("CAN_Write");
    g_CAN_GetErrorText = (pfn_CAN_GetErrorText)load("CAN_GetErrorText");
    printf("PCANBasic.dll loaded OK\n");
}

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
static constexpr uint32_t SETTLE_TIMEOUT_MS  = 8000;
static constexpr uint32_t RESEND_INTERVAL_MS = 500;

// Torque mode safety clamp — absolute max amps sent to either motor
// Raise only after you are confident the mechanics are clear
static constexpr float TORQUE_MAX_AMPS = 2.0f;

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
    CANBus(TPCANHandle ch, TPCANBaudrate baud) : ch_(ch) {
        TPCANStatus s = g_CAN_Initialize(ch_, baud, 0, 0, 0);
        if (s != PCAN_ERROR_OK) {
            char buf[256] = {};
            g_CAN_GetErrorText(s, 0x09, buf);
            throw std::runtime_error(std::string("CAN_Initialize: ") + buf);
        }
        printf("PCAN channel 0x%02X open at 1 Mbit/s\n", ch_);
    }
    ~CANBus() { g_CAN_Uninitialize(ch_); }

    void send(uint32_t id, const uint8_t data[8]) {
        TPCANMsg m{};
        m.ID = id; m.MSGTYPE = PCAN_MESSAGE_STANDARD; m.LEN = 8;
        memcpy(m.DATA, data, 8);
        if (g_CAN_Write(ch_, &m) != PCAN_ERROR_OK)
            throw std::runtime_error("CAN_Write failed");
    }

    bool recvLatest(uint32_t wantID, TPCANMsg& out, uint32_t timeoutMs = 25) {
        TPCANTimestamp ts{};
        bool got = false;
        DWORD deadline = GetTickCount() + timeoutMs;
        while (GetTickCount() < deadline) {
            TPCANMsg m{};
            TPCANStatus s = g_CAN_Read(ch_, &m, &ts);
            if (s == PCAN_ERROR_OK) {
                if (m.ID == wantID) { out = m; got = true; }
                continue;
            }
            if (s == PCAN_ERROR_QRCVEMPTY) {
                if (got) break;
                Sleep(1);
                continue;
            }
            throw std::runtime_error("CAN_Read error");
        }
        return got;
    }
private:
    TPCANHandle ch_;
};

// ── Decode reply ──────────────────────────────────────────
static MotorStatus decode(const TPCANMsg& m) {
    MotorStatus s;
    s.temperature_C = (int8_t)m.DATA[1];
    s.torque_A  = (int16_t)(m.DATA[2] | m.DATA[3] << 8) * 0.01f;
    s.speed_dps = (float)(int16_t)(m.DATA[4] | m.DATA[5] << 8);
    s.angle_deg = (float)(int16_t)(m.DATA[6] | m.DATA[7] << 8);
    s.valid     = true;
    return s;
}

// ── Low-level motor commands ──────────────────────────────
MotorStatus readStatus(CANBus& bus, uint8_t id) {
    uint8_t tx[8] = { CMD_READ_STATUS2, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, tx);
    TPCANMsg rx{};
    if (!bus.recvLatest(RX_BASE + id, rx, 30)) return {};
    if (rx.DATA[0] != CMD_READ_STATUS2)         return {};
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
    TPCANMsg rx{}; bus.recvLatest(RX_BASE + id, rx, 20); // drain echo
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
    TPCANMsg rx{};
    if (!bus.recvLatest(RX_BASE + id, rx, 30)) return {};
    if (rx.DATA[0] != CMD_TORQUE)               return {};
    return decode(rx);
}

void brakeRelease(CANBus& bus, uint8_t id) {
    uint8_t t[8] = { CMD_BRAKE_OPEN, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, t);
    TPCANMsg rx{}; bus.recvLatest(RX_BASE + id, rx, 30);
    Sleep(50); // give brake time to physically open
}

void brakeLock(CANBus& bus, uint8_t id) {
    uint8_t t[8] = { CMD_BRAKE_LOCK, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, t);
    TPCANMsg rx{}; bus.recvLatest(RX_BASE + id, rx, 30);
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
                      float pitchDeg, uint16_t pitchSpd,
                      uint32_t timeoutMs = SETTLE_TIMEOUT_MS) {
    printf("  -> roll=%.1f deg @ %u dps  |  pitch=%.1f deg @ %u dps\n",
           rollDeg, rollSpd, pitchDeg, pitchSpd);

    sendAbsPosition(bus, MOTOR_ROLL,  rollDeg,  rollSpd);
    Sleep(2);
    sendAbsPosition(bus, MOTOR_PITCH, pitchDeg, pitchSpd);

    DWORD deadline   = GetTickCount() + timeoutMs;
    DWORD lastResend = GetTickCount();

    while (GetTickCount() < deadline) {
        if (GetTickCount() - lastResend >= RESEND_INTERVAL_MS) {
            sendAbsPosition(bus, MOTOR_ROLL,  rollDeg,  rollSpd);
            Sleep(2);
            sendAbsPosition(bus, MOTOR_PITCH, pitchDeg, pitchSpd);
            lastResend = GetTickCount();
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
        Sleep(50);
    }
    printf("  -> timeout\n");
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

    DWORD deadline = GetTickCount() + durationMs;
    while (GetTickCount() < deadline) {
        MotorStatus r = sendTorque(bus, MOTOR_ROLL,  rollA);
        MotorStatus p = sendTorque(bus, MOTOR_PITCH, pitchA);
        printStatus("ROLL ", r);
        printStatus("PITCH", p);
        Sleep(20); // ~50 Hz
    }

    // Always zero torque first, then lock brake
    sendTorque(bus, MOTOR_ROLL,  0.f);
    sendTorque(bus, MOTOR_PITCH, 0.f);
    Sleep(20);
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
        printf("  %s home [speed_dps] [timeout_ms]\n", exe);
        printf("  %s move <roll_deg> <pitch_deg> [speed_dps] [timeout_ms]\n", exe);
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

        loadPCANDll();
        CANBus bus(PCAN_USBBUS1, PCAN_BAUD_1M);

        const std::string cmd = argv[1];

        // ── Commands available in both modes ──────────────
        if (cmd == "status") {
            printAllStatus(bus);
            return 0;
        }

        if (cmd == "stop") {
            printf("\n--- Stop ---\n");
            // Zero torque first in case we were in torque mode
            sendTorque(bus, MOTOR_ROLL,  0.f);
            sendTorque(bus, MOTOR_PITCH, 0.f);
            Sleep(20);
            motorStop(bus, MOTOR_ROLL);
            motorStop(bus, MOTOR_PITCH);
            printf("Done.\n");
            return 0;
        }

        // ── Position mode commands ────────────────────────
        if (CONTROL_MODE == ControlMode::Position) {

            if (cmd == "home") {
                uint16_t spd = argc >= 3 ? (uint16_t)parseUInt(argv[2], "speed_dps") : 30;
                uint32_t tmo = argc >= 4 ? parseUInt(argv[3], "timeout_ms") : SETTLE_TIMEOUT_MS;
                printf("\n--- Home ---\n");
                moveBothPosition(bus, 0.f, spd, 0.f, spd, tmo);
                printf("\n--- Final status ---\n");
                printAllStatus(bus);

            } else if (cmd == "move") {
                if (argc < 4) { printUsage(argv[0]); return 1; }
                float    roll  = parseFloat(argv[2], "roll_deg");
                float    pitch = parseFloat(argv[3], "pitch_deg");
                uint16_t spd   = argc >= 5 ? (uint16_t)parseUInt(argv[4], "speed_dps") : 30;
                uint32_t tmo   = argc >= 6 ? parseUInt(argv[5], "timeout_ms") : SETTLE_TIMEOUT_MS;
                printf("\n--- Move ---\n");
                moveBothPosition(bus, roll, spd, pitch, spd, tmo);
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