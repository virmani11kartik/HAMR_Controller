// canfilter.c
// Build:  gcc -Wall -O2 canfilter.c -o canfilter
// Usage:
//   sudo ./canfilter                  # defaults: ifname=can0, motor IDs 1 and 2
//   sudo ./canfilter can0 1 2         # explicit
//   sudo ./canfilter can0 3           # single motor (ID 3)
// Notes:
//   - Filters are set to RX IDs: 0x240 + motor_id (standard 11-bit).
//   - Prints continuously with timestamps until Ctrl+C.

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <signal.h>
#include <errno.h>
#include <sys/time.h>

#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>

static volatile int keep_running = 1;
static void on_sigint(int sig) { (void)sig; keep_running = 0; }

static void print_frame(const struct can_frame *f) {
    struct timeval tv;
    gettimeofday(&tv, NULL);
    double ts = tv.tv_sec + tv.tv_usec / 1e6;

    // Indicate extended/remote frames if ever seen (RMD uses standard frames)
    int is_eff = (f->can_id & CAN_EFF_FLAG) != 0;
    int is_rtr = (f->can_id & CAN_RTR_FLAG) != 0;
    canid_t id = f->can_id & (is_eff ? CAN_EFF_MASK : CAN_SFF_MASK);

    printf("[%.6f] %s 0x%03X [%d]%s  ",
           ts,
           is_eff ? "EFF" : "SFF",
           id,
           f->can_dlc,
           is_rtr ? " RTR" : "");

    for (int i = 0; i < f->can_dlc && !is_rtr; i++)
        printf("%02X ", f->data[i]);
    printf("\n");
    fflush(stdout);
}

int main(int argc, char **argv) {
    const char *ifname = "can0";
    int motor_ids[2] = {1, 2};
    int n_ids = 2;

    // Parse args
    // ./canfilter [ifname] [id1] [id2]
    if (argc >= 2) ifname = argv[1];
    if (argc >= 3) {
        motor_ids[0] = atoi(argv[2]);
        n_ids = 1;
    }
    if (argc >= 4) {
        motor_ids[1] = atoi(argv[3]);
        n_ids = 2;
    }

    // Compute RX CAN IDs (0x240 + ID)
    canid_t rx_ids[2];
    for (int i = 0; i < n_ids; i++) {
        if (motor_ids[i] < 1 || motor_ids[i] > 32) {
            fprintf(stderr, "Motor ID %d out of range (1..32)\n", motor_ids[i]);
            return 2;
        }
        rx_ids[i] = 0x240 + motor_ids[i];
    }

    printf("CAN listener on %s for %d motor(s):\n", ifname, n_ids);
    for (int i = 0; i < n_ids; i++)
        printf("  Motor ID %d -> RX 0x%03X\n", motor_ids[i], rx_ids[i]);

    // Open socket
    int s = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (s < 0) { perror("socket"); return 1; }

    // Bind to interface
    struct ifreq ifr;
    memset(&ifr, 0, sizeof(ifr));
    strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(s, SIOCGIFINDEX, &ifr) < 0) { perror("SIOCGIFINDEX"); close(s); return 1; }

    struct sockaddr_can addr = {0};
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(s, (struct sockaddr *)&addr, sizeof(addr)) < 0) { perror("bind"); close(s); return 1; }

    // Install filters for the chosen motor reply IDs
    struct can_filter flt[2];
    for (int i = 0; i < n_ids; i++) {
        flt[i].can_id   = rx_ids[i];
        flt[i].can_mask = CAN_SFF_MASK; // exact 11-bit match
    }
    if (setsockopt(s, SOL_CAN_RAW, CAN_RAW_FILTER, flt, sizeof(struct can_filter) * n_ids) < 0) {
        perror("setsockopt(CAN_RAW_FILTER)");
        close(s);
        return 1;
    }

    // Optional: enable CAN FD reception off (we’re using classic CAN)
    int enable_canfd = 0;
    setsockopt(s, SOL_CAN_RAW, CAN_RAW_FD_FRAMES, &enable_canfd, sizeof(enable_canfd));

    // Handle Ctrl+C
    signal(SIGINT, on_sigint);

    printf("Listening… (Ctrl+C to quit)\n");

    // Receive loop
    while (keep_running) {
        struct can_frame f;
        ssize_t n = read(s, &f, sizeof(f));
        if (n < 0) {
            if (errno == EINTR) break;
            perror("read");
            break;
        }
        if ((size_t)n < sizeof(struct can_frame)) {
            fprintf(stderr, "short read (%zd)\n", n);
            continue;
        }
        print_frame(&f);
    }

    close(s);
    printf("Bye.\n");
    return 0;
}
