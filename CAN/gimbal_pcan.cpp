#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <cstdio>
#include <cmath>
#include <stdexcept>
#include <string>
#include <windows.h>

// ── Minimal PCAN types (no PCANBasic.h) ──────────────────
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

static constexpr uint8_t CMD_READ_STATUS2   = 0x9C;
static constexpr uint8_t CMD_ABS_POSITION   = 0xA4;
static constexpr uint8_t CMD_MOTOR_STOP     = 0x81;
static constexpr uint8_t CMD_MOTOR_SHUTDOWN = 0x80;

// Settle thresholds
static constexpr float    SETTLE_SPEED_DPS  = 3.0f;
static constexpr float    SETTLE_ANGLE_DEG  = 1.0f;
static constexpr uint32_t SETTLE_TIMEOUT_MS = 8000;
static constexpr uint32_t RESEND_INTERVAL_MS = 500; // re-assert target

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
            char buf[256]={};
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

    // Drain RX buffer, return the LATEST frame matching wantID.
    // Keeps reading until buffer is empty then returns best result.
    bool recvLatest(uint32_t wantID, TPCANMsg& out, uint32_t timeoutMs = 25) {
        TPCANTimestamp ts{};
        bool got = false;
        DWORD deadline = GetTickCount() + timeoutMs;
        while (GetTickCount() < deadline) {
            TPCANMsg m{};
            TPCANStatus s = g_CAN_Read(ch_, &m, &ts);
            if (s == PCAN_ERROR_OK) {
                if (m.ID == wantID) { out = m; got = true; }
                continue; // keep draining for freshest frame
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

// ── Decode reply bytes ────────────────────────────────────
static MotorStatus decode(const TPCANMsg& m) {
    MotorStatus s;
    s.temperature_C = (int8_t)m.DATA[1];
    s.torque_A  = (int16_t)(m.DATA[2] | m.DATA[3]<<8) * 0.01f;
    s.speed_dps = (float)(int16_t)(m.DATA[4] | m.DATA[5]<<8);
    s.angle_deg = (float)(int16_t)(m.DATA[6] | m.DATA[7]<<8);
    s.valid     = true;
    return s;
}

// ── Motor helpers ─────────────────────────────────────────
MotorStatus readStatus(CANBus& bus, uint8_t id) {
    uint8_t tx[8] = { CMD_READ_STATUS2, 0,0,0,0,0,0,0 };
    bus.send(TX_BASE + id, tx);
    TPCANMsg rx{};
    if (!bus.recvLatest(RX_BASE + id, rx, 30)) return {};
    if (rx.DATA[0] != CMD_READ_STATUS2)         return {};
    return decode(rx);
}

// Fire a position command and drain the echo — does NOT block waiting for motion
void sendAbsPosition(CANBus& bus, uint8_t id, float deg, uint16_t maxDps) {
    int32_t ac = (int32_t)std::lroundf(deg * 100.f);
    uint8_t tx[8] = {
        CMD_ABS_POSITION, 0x00,
        (uint8_t)maxDps,     (uint8_t)(maxDps>>8),
        (uint8_t)ac,         (uint8_t)(ac>>8),
        (uint8_t)(ac>>16),   (uint8_t)(ac>>24)
    };
    bus.send(TX_BASE + id, tx);
    TPCANMsg rx{}; bus.recvLatest(RX_BASE + id, rx, 20); // drain echo
}

void motorStop    (CANBus& bus, uint8_t id) { uint8_t t[8]={CMD_MOTOR_STOP,    0,0,0,0,0,0,0}; bus.send(TX_BASE+id,t); }
void motorShutdown(CANBus& bus, uint8_t id) { uint8_t t[8]={CMD_MOTOR_SHUTDOWN,0,0,0,0,0,0,0}; bus.send(TX_BASE+id,t); }

void printStatus(const char* lbl, const MotorStatus& s) {
    if (!s.valid) { printf("[%-5s] no reply\n", lbl); return; }
    printf("[%-5s] %3d C  torque=%+6.2f A  speed=%+7.1f dps  angle=%+8.1f deg\n",
           lbl, s.temperature_C, s.torque_A, s.speed_dps, s.angle_deg);
}

void printAllStatus(CANBus& bus) {
    printStatus("ROLL ", readStatus(bus, MOTOR_ROLL));
    printStatus("PITCH", readStatus(bus, MOTOR_PITCH));
}

void moveBoth(CANBus& bus,
              float rollDeg,  uint16_t rollSpd,
              float pitchDeg, uint16_t pitchSpd,
              uint32_t timeoutMs = SETTLE_TIMEOUT_MS) {

    printf("  -> roll=%.1f deg @ %u dps  |  pitch=%.1f deg @ %u dps\n",
           rollDeg, rollSpd, pitchDeg, pitchSpd);

    // Fire both commands
    sendAbsPosition(bus, MOTOR_ROLL,  rollDeg,  rollSpd);
    Sleep(2);
    sendAbsPosition(bus, MOTOR_PITCH, pitchDeg, pitchSpd);

    DWORD deadline   = GetTickCount() + timeoutMs;
    DWORD lastResend = GetTickCount();

    while (GetTickCount() < deadline) {

        // Re-assert target periodically so watchdog doesn't trigger
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

        if (rollDone && pitchDone) {
            printf("  -> settled\n");
            return;
        }

        Sleep(50); // poll at ~20 Hz
    }
    printf("  -> timeout\n");
}

void printUsage(const char* exe) {
    printf("Usage:\n");
    printf("  %s status\n", exe);
    printf("  %s home [speed_dps] [timeout_ms]\n", exe);
    printf("  %s move <roll_deg> <pitch_deg> [speed_dps] [timeout_ms]\n", exe);
    printf("  %s stop\n", exe);
    printf("\nExamples:\n");
    printf("  %s home\n", exe);
    printf("  %s move 15 -10 30\n", exe);
    printf("  %s status\n", exe);
}

float parseFloatArg(const char* text, const char* name) {
    char* end = nullptr;
    const float value = std::strtof(text, &end);
    if (end == text || (end != nullptr && *end != '\0'))
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    return value;
}

uint32_t parseUIntArg(const char* text, const char* name) {
    char* end = nullptr;
    const unsigned long value = std::strtoul(text, &end, 10);
    if (end == text || (end != nullptr && *end != '\0'))
        throw std::runtime_error(std::string("Invalid ") + name + ": " + text);
    return static_cast<uint32_t>(value);
}

// ── Main ──────────────────────────────────────────────────
int main(int argc, char** argv) {
    try {
        if (argc < 2) {
            printUsage(argv[0]);
            return 1;
        }

        loadPCANDll();
        CANBus bus(PCAN_USBBUS1, PCAN_BAUD_1M);
        const std::string command = argv[1];

        if (command == "status") {
            printAllStatus(bus);
        } else if (command == "home") {
            const uint16_t speed =
                argc >= 3 ? static_cast<uint16_t>(parseUIntArg(argv[2], "speed_dps")) : 30;
            const uint32_t timeout =
                argc >= 4 ? parseUIntArg(argv[3], "timeout_ms") : SETTLE_TIMEOUT_MS;
            printf("\n--- Home: 0 / 0 deg ---\n");
            moveBoth(bus, 0.f, speed, 0.f, speed, timeout);
            printf("\n--- Final status ---\n");
            printAllStatus(bus);
        } else if (command == "move") {
            if (argc < 4) {
                printUsage(argv[0]);
                return 1;
            }
            const float rollDeg = parseFloatArg(argv[2], "roll_deg");
            const float pitchDeg = parseFloatArg(argv[3], "pitch_deg");
            const uint16_t speed =
                argc >= 5 ? static_cast<uint16_t>(parseUIntArg(argv[4], "speed_dps")) : 30;
            const uint32_t timeout =
                argc >= 6 ? parseUIntArg(argv[5], "timeout_ms") : SETTLE_TIMEOUT_MS;

            printf("\n--- Move ---\n");
            moveBoth(bus, rollDeg, speed, pitchDeg, speed, timeout);
            printf("\n--- Final status ---\n");
            printAllStatus(bus);
        } else if (command == "stop") {
            printf("\n--- Stop ---\n");
            motorStop(bus, MOTOR_ROLL);
            motorStop(bus, MOTOR_PITCH);
            printf("Done.\n");
        } else {
            printUsage(argv[0]);
            return 1;
        }

    } catch (const std::exception& e) {
        fprintf(stderr, "ERROR: %s\n", e.what());
        return 1;
    }
    return 0;
}
