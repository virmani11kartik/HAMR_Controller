#include <iostream>
#include <iomanip>
#include <cstring>
#include <cstdlib>
#include <cerrno>

#include <unistd.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <sys/socket.h>

#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>

int main() {
    const char *ifname = "can0";
    int sock;
    struct ifreq ifr;
    struct sockaddr_can addr;

    // Create CAN RAW socket
    if ((sock = socket(PF_CAN, SOCK_RAW, CAN_RAW)) < 0) {
        perror("socket");
        return 1;
    }

    // Locate interface index
    std::strncpy(ifr.ifr_name, ifname, IFNAMSIZ - 1);
    if (ioctl(sock, SIOCGIFINDEX, &ifr) < 0) {
        perror("ioctl");
        return 1;
    }

    // Bind socket to CAN interface
    addr.can_family  = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(sock, (struct sockaddr *)&addr, sizeof(addr)) < 0) {
        perror("bind");
        return 1;
    }

    std::cout << "Listening on " << ifname << " for ID 0x100…" << std::endl;

    struct can_frame frame;
    while (true) {
        int nbytes = read(sock, &frame, sizeof(frame));
        if (nbytes < 0) {
            perror("read");
            break;
        }

        if ((frame.can_id & CAN_SFF_MASK) == 0x100 && frame.can_dlc >= 6) {
            // Extract little-endian int16_t values
            int16_t r_cd = frame.data[0] | (frame.data[1] << 8);
            int16_t p_cd = frame.data[2] | (frame.data[3] << 8);
            int16_t y_cd = frame.data[4] | (frame.data[5] << 8);

            float roll  = r_cd / 100.0f;
            float pitch = p_cd / 100.0f;
            float yaw   = y_cd / 100.0f;

            std::cout << std::fixed << std::setprecision(2)
                      << "RPY(deg): " << roll << ", "
                      << pitch << ", "
                      << yaw << std::endl;
        }
    }

    close(sock);
    return 0;
}
