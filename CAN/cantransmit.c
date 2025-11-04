// cantransmit.c
// Build:  gcc -Wall -O2 cantransmit.c -o cantransmit
// Usage:
//   # Read multi-turn angle (default 0x92):
//   sudo ./cantransmit can0 1 read-angle
//
//   # Send a raw 8-byte frame (hex bytes, no 0x prefix needed):
//   sudo ./cantransmit can0 2 raw 92 00 00 00 00 00 00 00
//
// Notes:
//   - MyActuator RMD series typically uses standard 11-bit IDs:
//       TX (host->motor) = 0x140 + ID   e.g., ID=1 => 0x141
//       RX (motor->host) = 0x240 + ID   e.g., ID=1 => 0x241
//   - This tool *transmits only*. Pair it with your canfilter listener to see replies.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>
#include <stdint.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static int send_can(int s, canid_t can_id, const uint8_t *data, uint8_t dlc) {
    struct can_frame f = {0};
    f.can_id  = can_id;    // standard 11-bit
    f.can_dlc = dlc;
    memcpy(f.data, data, dlc);
    int n = write(s, &f, sizeof(f));
    if (n != (int)sizeof(f)) {
        perror("write");
        return -1;
    }
    return 0;
}

static void usage(const char *prog) {
    fprintf(stderr,
        "Usage:\n"
        "  sudo %s <ifname> <motor_id> read-angle\n"
        "  sudo %s <ifname> <motor_id> raw <b0> <b1> <b2> <b3> <b4> <b5> <b6> <b7>\n"
        "\nExamples:\n"
        "  sudo %s can0 1 read-angle\n"
        "  sudo %s can0 2 raw 92 00 00 00 00 00 00 00\n",
        prog, prog, prog, prog
    );
}

int main(int argc, char **argv) {
    if (argc < 4) {
        usage(argv[0]);
        return 1;
    }

    const char *ifname = argv[1];
    int motor_id = atoi(argv[2]);
    const char *mode = argv[3];

    if (motor_id < 1 || motor_id > 32) {
        fprintf(stderr, "motor_id must be in 1..32\n");
        return 2;
    }

    // Compute TX arbitration ID for this motor
    canid_t tx_id = 0x140 + motor_id;   // host -> motor

    // Open CAN socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    // Bind to interface
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) {
        perror("SIOCGIFINDEX");
        close(s);
        return 1;
    }

    struct sockaddr_can addr = {0};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        close(s);
        return 1;
    }

    // Prepare payload
    uint8_t payload[8] = {0};
    int dlc = 8;

    if (strcmp(mode, "read-angle") == 0) {
        // Typical RMD "read multi-turn angle" command byte is 0x92 with rest zeros.
        payload[0] = 0x92;
        printf("TX if=%s ID=%d (0x%03X): [", ifname, motor_id, tx_id);
        for (int i=0;i<dlc;i++) printf("%02X%s", payload[i], (i==dlc-1?"]\n":" "));
        if (send_can(s, tx_id, payload, dlc) != 0) {
            fprintf(stderr, "failed to send read-angle\n");
            close(s);
            return 1;
        }
        printf("Sent read-angle. Use your canfilter or candump to view the reply (0x%03X).\n", 0x240 + motor_id);

    } else if (strcmp(mode, "raw") == 0) {
        if (argc != 12) {
            fprintf(stderr, "raw mode requires 8 hex byte args.\n");
            usage(argv[0]);
            close(s);
            return 1;
        }
        for (int i = 0; i < 8; i++) {
            // accept hex with or without 0x
            char *endp = NULL;
            unsigned long v = strtoul(argv[4+i], &endp, 16);
            if (*argv[4+i] == '\0' || *endp != '\0' || v > 0xFF) {
                fprintf(stderr, "invalid byte %s (expect hex 00..FF)\n", argv[4+i]);
                close(s);
                return 1;
            }
            payload[i] = (uint8_t)v;
        }
        printf("TX if=%s ID=%d (0x%03X): [", ifname, motor_id, tx_id);
        for (int i=0;i<dlc;i++) printf("%02X%s", payload[i], (i==dlc-1?"]\n":" "));
        if (send_can(s, tx_id, payload, dlc) != 0) {
            fprintf(stderr, "failed to send raw payload\n");
            close(s);
            return 1;
        }
        printf("Sent raw payload. Expected reply on 0x%03X (if the command has a response).\n", 0x240 + motor_id);

    } else {
        fprintf(stderr, "Unknown mode: %s\n", mode);
        usage(argv[0]);
        close(s);
        return 1;
    }

    close(s);
    return 0;
}
