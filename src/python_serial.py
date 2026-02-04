# @file python_serial.py
# @author Samor / Gemini CLI
# @brief Professional multi-motor telemetry visualizer for AK-Series.

import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re
import collections

# --- Configuration ---
SERIAL_PORT = "/dev/ttyUSB0"  # Update this to your ESP32 port
BAUD_RATE = 115200
MAX_POINTS = 200  # Horizontal history (points)

# --- State ---
# Structure: { motor_id: { 'pos': deque, 'spd': deque, 'cur': deque, 'time': deque, 't_count': int } }
motors_data = {}

# Open serial connection
try:
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=0.1)
    print(f"Connected to {SERIAL_PORT}")
except Exception as e:
    print(f"Failed to connect: {e}")
    exit()


def parse_line(line):
    """
    Parses format: "Motor [1] -> Pos: 180.0 deg | Spd: 0 RPM | Cur: 0.00 A"
    """
    pattern = r"Motor \[(\d+)\] -> Pos: ([\d\.\-]+) deg \| Spd: ([\d\.\-]+) RPM \| Cur: ([\d\.\-]+) A"
    match = re.search(pattern, line)
    if match:
        return {
            "id": int(match.group(1)),
            "pos": float(match.group(2)),
            "spd": float(match.group(3)),
            "cur": float(match.group(4)),
        }
    return None


def update_plot(frame):
    global motors_data

    # 1. Ingest all available Serial data
    while ser.in_waiting > 0:
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
            data = parse_line(line)
            if data:
                m_id = data["id"]
                # Initialize motor if new
                if m_id not in motors_data:
                    motors_data[m_id] = {
                        "pos": collections.deque(maxlen=MAX_POINTS),
                        "spd": collections.deque(maxlen=MAX_POINTS),
                        "cur": collections.deque(maxlen=MAX_POINTS),
                        "time": collections.deque(maxlen=MAX_POINTS),
                        "t_count": 0,
                    }

                # Append data
                m = motors_data[m_id]
                m["pos"].append(data["pos"])
                m["spd"].append(data["spd"])
                m["cur"].append(data["cur"])
                m["time"].append(m["t_count"])
                m["t_count"] += 1
        except Exception:
            continue

    if not motors_data:
        return

    # 2. Dynamic UI Management
    m_ids = sorted(motors_data.keys())
    num_motors = len(m_ids)

    # If number of motors changed, we need to rebuild subplots (rare but possible)
    if not hasattr(update_plot, "last_count") or update_plot.last_count != num_motors:
        plt.clf()
        update_plot.axs = fig.subplots(3, num_motors, sharex="col", squeeze=False)
        update_plot.last_count = num_motors
        plt.tight_layout(pad=2.0)

    # 3. Update Axes
    for col, m_id in enumerate(m_ids):
        m = motors_data[m_id]
        labels = ["Pos (deg)", "Spd (RPM)", "Cur (A)"]
        keys = ["pos", "spd", "cur"]
        colors = ["#1f77b4", "#ff7f0e", "#2ca02c"]

        for row in range(3):
            ax = update_plot.axs[row, col]
            ax.clear()
            ax.plot(list(m["time"]), list(m[keys[row]]), color=colors[row])
            ax.grid(True, alpha=0.3)

            if row == 0:
                ax.set_title(f"Motor ID {m_id}")
            if col == 0:
                ax.set_ylabel(labels[row])
            if row == 2:
                ax.set_xlabel("Samples")


# Setup Figure
fig = plt.figure(figsize=(12, 8))
ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)

try:
    plt.show()
finally:
    ser.close()
    print("Serial closed.")
