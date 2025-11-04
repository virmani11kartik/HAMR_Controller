// build:  gcc -Wall -O2 can_rmd_demo.c -o can_rmd_demo
// usage:  sudo ./can_rmd_demo can0 1           # interface, motor_id=1
// notes:  bring interface up first:
//         sudo ip link set can0 up type can bitrate 1000000

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/select.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <stdint.h>   // <--- ADD THIS LINE

#include <linux/can.h>
#include <linux/can/raw.h>

static int send_can(int s, canid_t can_id, const uint8_t *data, uint8_t dlc) {
    struct can_frame f = {0};
    f.can_id  = can_id;        // 11-bit ID for RMD series (no CAN_EFF_FLAG)
    f.can_dlc = dlc;
    memcpy(f.data, data, dlc);
    int n = write(s, &f, sizeof(f));
    if (n != sizeof(f)) {
        perror("write");
        return -1;
    }
    return 0;
}

int main(int argc, char **argv) {
    if (argc < 3) {
        fprintf(stderr, "usage: %s <ifname> <motor_id>\n", argv[0]);
        return 1;
    }

    const char *ifname = argv[1];     // e.g., "can0"
    int motor_id = atoi(argv[2]);     // 1..8 typically for RMD

    // Common RMD arbitration IDs (verify in your datasheet!)
    canid_t tx_id = 0x140 + 0x01 + (motor_id - 1);  // often 0x141..0x148
    canid_t rx_id = 0x240 + 0x01 + (motor_id - 1);  // often 0x241..0x248

    int s;
    if ((s = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return 1;
    }

    // Optional: set a filter so we only receive this motor’s replies.
    struct can_filter filt[1];
    filt[0].can_id   = rx_id;
    filt[0].can_mask = CAN_SFF_MASK; // match full 11-bit ID
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, &filt, sizeof(filt)) < 0) {
        perror("setsockopt(FILTER)");
        close(s);
        return 1;
    }

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

    // ---------------------------
    // Example command to send:
    // Many RMD motors support a "Read multi-turn angle" with cmd byte 0x92.
    // The frame is 8 bytes total. Often the rest are zeros.
    // Confirm in your exact L-7015 manual!
    // ---------------------------
    uint8_t cmd_read_angle[8] = {0};
    cmd_read_angle[0] = 0x92;      // <--- check this in your datasheet

    if (send_can(s, tx_id, cmd_read_angle, 8) != 0) {
        fprintf(stderr, "Failed to send command\n");
        close(s);
        return 1;
    }
    printf("Sent read-angle request on 0x%03X, waiting for reply 0x%03X…\n",
           tx_id, rx_id);

    // Wait up to 200 ms for a reply
    fd_set rfds;
    FD_ZERO(&rfds);
    FD_SET(s, &rfds);
    struct timeval tv = {0};
    tv.tv_sec  = 0;
    tv.tv_usec = 200000;

    int rv = select(s + 1, &rfds, NULL, NULL, &tv);
    if (rv < 0) {
        perror("select");
        close(s);
        return 1;
    } else if (rv == 0) {
        fprintf(stderr, "Timeout waiting for reply.\n");
        close(s);
        return 1;
    }

    // Read one reply frame
    struct can_frame frame;
    int nbytes = read(s, &frame, sizeof(frame));
    if (nbytes < 0) {
        perror("read");
        close(s);
        return 1;
    }

    printf("RX 0x%03X [%d]:", frame.can_id, frame.can_dlc);
    for (int i = 0; i < frame.can_dlc; i++) printf(" %02X", frame.data[i]);
    printf("\n");

    // TODO: parse the reply per your manual.
    // Angle replies often pack angle ticks into bytes 0..3 or similar.
    // Convert to degrees/radians as specified by the vendor.

    close(s);
    return 0;
}
