import pygame
import socket
import serial
import time
import select
import csv
import datetime
import re
import matplotlib.pyplot as plt

# ========= Transport toggles =========
USE_SERIAL = True   # True -> talk to ESP32 over USB serial
USE_UDP    = False  # True -> keep UDP send/recv as well (fallback)

# ========= Serial config =========
SER_PORT = "/dev/ttyACM0"  # adjust as needed
SER_BAUD = 115200
SER_TIMEOUT = 0.0           # non-blocking

# ========= UDP config (if used) =========
ESP32_IP   = "192.168.4.1"
ESP32_PORT = 12345

# ---------- Setup transports ----------
ser = None
if USE_SERIAL:
    ser = serial.Serial(SER_PORT, SER_BAUD, timeout=SER_TIMEOUT)
    # give the ESP a moment after opening the port (optional)
    time.sleep(0.5)

sock = None
if USE_UDP:
    sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
    sock.setblocking(False)

# ---------- Controller ----------
pygame.init()
pygame.joystick.init()
if pygame.joystick.get_count() == 0:
    print("No controller detected.")
    raise SystemExit
joystick = pygame.joystick.Joystick(0)
joystick.init()
print(f"Connected to controller: {joystick.get_name()}")

# ---------- CSV + Plot ----------
csv_file = open("controller_and_pose_log.csv", mode="w", newline="")
csv_writer = csv.writer(csv_file)
csv_writer.writerow([
    "timestamp",
    "LX", "LY", "RX", "RY", "LT", "RT", "A", "B",
    "pose_x", "pose_x_std", "pose_y", "pose_y_std", "pose_theta", "pose_theta_std"
])

plt.ion()
fig, ax = plt.subplots()
trajectory_x, trajectory_y = [], []
traj_plot, = ax.plot([], [], 'b.-', label='Trajectory')
ax.set_xlabel("X [m]"); ax.set_ylabel("Y [m]")
ax.set_title("Real-Time Robot Trajectory")
ax.grid(True); ax.legend()
plt.show()
print("Plot ready. Entering loop…")

# Pose line regex (same as your UDP parser)
POSE_RE = re.compile(
    r"X:\s*([-\d\.]+)\s*±\s*([-\d\.]+)\s*m,\s*Y:\s*([-\d\.]+)\s*±\s*([-\d\.]+)\s*m,\s*Theta:\s*([-\d\.]+)\s*±\s*([-\d\.]+)"
)

def try_parse_pose_line(line):
    m = POSE_RE.search(line)
    if not m:
        return None
    return tuple(map(float, m.groups()))  # (x, x_std, y, y_std, theta, theta_std)

try:
    last_plot = time.time()
    while True:
        updated = False
        pygame.event.pump()

        # Sticks & triggers (keep your original mapping)
        left_x  = joystick.get_axis(0)
        left_y  = -joystick.get_axis(1)
        right_x = joystick.get_axis(3)
        right_y = -joystick.get_axis(4)
        left_trigger  = joystick.get_axis(2)
        right_trigger = joystick.get_axis(5)

        a = joystick.get_button(0)
        b = joystick.get_button(1)
        x = joystick.get_button(2)
        y = joystick.get_button(3)

        msg = (
            f"LX:{left_x:.2f} LY:{left_y:.2f} "
            f"RX:{right_x:.2f} RY:{right_y:.2f} "
            f"LT:{left_trigger:.2f} RT:{right_trigger:.2f} "
            f"A:{a} B:{b} X:{x} Y:{y}\n"
        )

        # -------- Send to ESP --------
        if USE_SERIAL and ser is not None:
            # Non-blocking write is fine for short lines
            ser.write(msg.encode('utf-8'))

        if USE_UDP and sock is not None:
            sock.sendto(msg.encode('utf-8'), (ESP32_IP, ESP32_PORT))

        timestamp = datetime.datetime.now().isoformat(timespec='milliseconds')
        pose_x = pose_x_std = pose_y = pose_y_std = pose_theta = pose_theta_std = None

        # -------- Receive from ESP (prefer serial when enabled) --------
        # if USE_SERIAL and ser is not None:
        #     # Non-blocking read: drain all available lines
        #     # (ESP prints pose often; we'll just parse the latest)
        #     while ser.in_waiting > 0:
        #         try:
        #             line = ser.readline().decode('utf-8', errors='ignore').strip()
        #         except Exception:
        #             break
        #         if not line:
        #             continue
        #         parsed = try_parse_pose_line(line)
        #         if parsed:
        #             (pose_x, pose_x_std, pose_y, pose_y_std, pose_theta, pose_theta_std) = parsed
        #             trajectory_x.append(pose_x); trajectory_y.append(pose_y)
        #             updated = True
        #             # print("SER pose:", line)  # optional debug

        if USE_UDP and sock is not None:
            ready = select.select([sock], [], [], 0)
            if ready[0]:
                data, addr = sock.recvfrom(1024)
                line = data.decode('utf-8', errors='ignore').strip()
                parsed = try_parse_pose_line(line)
                if parsed:
                    (pose_x, pose_x_std, pose_y, pose_y_std, pose_theta, pose_theta_std) = parsed
                    trajectory_x.append(pose_x); trajectory_y.append(pose_y)
                    updated = True
                    # print("UDP pose:", line)  # optional debug

        # -------- Plot update (throttled) --------
        if updated or (time.time() - last_plot) > 0.1:
            traj_plot.set_data(trajectory_x, trajectory_y)
            ax.relim(); ax.autoscale_view()
            plt.pause(0.001)
            last_plot = time.time()

        # -------- Log CSV --------
        csv_writer.writerow([
            timestamp,
            left_x, left_y, right_x, right_y, left_trigger, right_trigger, a, b,
            pose_x, pose_x_std, pose_y, pose_y_std, pose_theta, pose_theta_std
        ])
        csv_file.flush()

        time.sleep(0.02)  # ~50 Hz loop

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    if ser:  ser.close()
    if sock: sock.close()
    pygame.quit()
    csv_file.close()
    plt.close(fig)
