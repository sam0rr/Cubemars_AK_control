import serial
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import re

# Serial port configuration
serial_port = "/dev/ttyUSB0"  # Update with your Arduino's serial port
baud_rate = 115200

# Open serial connection
ser = serial.Serial(serial_port, baud_rate, timeout=1)

# Data storage
pos_x_data = []
time_data = []

# Time counter
time_counter = 0

def parse_data(line):
    """
    Parse the incoming data packet from Arduino.
    Expected format: "SEND pos_x, vel_x, cur_x, pos_y, vel_y, cur_y, pos_z, vel_z, cur_z"
    """
    try:
        # Extract the numbers using regular expressions
        match = re.match(r"SEND ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+), ([\d\.\-]+)", line)
        if match:
            return [float(value) for value in match.groups()]
    except Exception as e:
        print(f"Error parsing data: {e}")
    return None

def update_plot(frame):
    global time_counter
    # Read line from serial port
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        data = parse_data(line)
        if data:
            pos_x = data[0]  # Extract pos_x
            pos_x_data.append(pos_x)
            time_data.append(time_counter)
            time_counter += 1

            # Keep only the last 100 points for smoother plotting
            if len(pos_x_data) > 100:
                pos_x_data.pop(0)
                time_data.pop(0)

            # Update the plot
            ax.clear()
            ax.plot(time_data, pos_x_data, label="Position X")
            ax.set_title("Real-Time Position X")
            ax.set_xlabel("Time (s)")
            ax.set_ylabel("Position (m)")
            ax.legend()
            ax.grid()

# Set up the plot
fig, ax = plt.subplots()
ani = FuncAnimation(fig, update_plot, interval=100)

# Show the plot
plt.show()

# Close the serial connection on exit
ser.close()
