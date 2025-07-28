import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import serial.tools.list_ports
import numpy as np

def get_arduino_port():
    """Find the Arduino port automatically"""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "Arduino" in port.description or "CH340" in port.description or "USB Serial" in port.description:
            return port.device
    return None

# Use the specific port since we know that's our port
serial_port = 'COM7'
print(f"Using port: {serial_port}")

baud_rate = 9600  # Match this with your Arduino baud rate

try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Connected to {serial_port}")
except serial.SerialException as e:
    print(f"Error opening port {serial_port}: {e}")
    exit(1)

# Ask user for their total body mass and calculate effective mass
total_mass = float(input("Enter your total body mass in kilograms: "))
effective_mass = 0.20 * total_mass  # 20% of total body mass

# Lists to store the accelerometer data
time_data = []
ax_data = []
ay_data = []
az_data = []
fx_data = []
fy_data = []
fz_data = []
vx_data = [0]  # Start with initial velocity as 0
vy_data = [0]
vz_data = [0]
x_data = [0]  # Start with initial position as 0
y_data = [0]
z_data = [0]
prev_ax = prev_ay = prev_az = 0  # Initial previous acceleration values
vx = vy = vz = 0  # Initial velocities
x = y = z = 0  # Initial positions

# Capture data for a certain amount of time (in seconds)
duration = 5  # Record data for 5 seconds
start_time = time.time()
prev_time = start_time

try:
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            current_time = time.time()
            dt = current_time - prev_time  # Calculate actual time elapsed since last reading
            prev_time = current_time
            # Read one line of data from the serial port
            line = ser.readline().decode('utf-8').strip()  # Decode byte to string and strip newline
            try:
                # Split the line into the three acceleration values (Ax, Ay, Az)
                ax, ay, az = map(float, line.split(','))
                # Adjust for any sensor offset or calibration
                ax -= 0.04
                ay += 0.08
                az += 0.97
                # Store the data
                time_data.append(current_time - start_time)
                ax_data.append(ax)
                ay_data.append(ay)
                az_data.append(az)

                # Calculate force (F = ma)
                fx = ax * effective_mass
                fy = ay * effective_mass
                fz = az * effective_mass
                fx_data.append(fx)
                fy_data.append(fy)
                fz_data.append(fz)

                # Trapezoidal integration for velocity
                vx += (prev_ax + ax) / 2 * dt
                vy += (prev_ay + ay) / 2 * dt
                vz += (prev_az + az) / 2 * dt
                prev_ax, prev_ay, prev_az = ax, ay, az  # Store current accelerations as previous for next iteration

                vx_data.append(vx)
                vy_data.append(vy)
                vz_data.append(vz)

                # Trapezoidal integration for position
                x += (vx_data[-2] + vx) / 2 * dt
                y += (vy_data[-2] + vy) / 2 * dt
                z += (vz_data[-2] + vz) / 2 * dt

                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

            except ValueError as e:
                print(f"Invalid data received: {line}")
                continue
except KeyboardInterrupt:
    print("\nData collection stopped by user")
finally:
    # Close the serial connection
    ser.close()
    print("Serial connection closed")

# Only plot if we have collected data
if time_data:
    # Create figure with subplots
    fig = plt.figure(figsize=(15, 7))
    
    # 3D position plot
    ax1 = fig.add_subplot(141, projection='3d')
    ax1.plot(x_data, y_data, z_data, 'b-', label='Position')
    ax1.set_title('3D Position Trajectory')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Z Position (m)')
    ax1.grid(True)
    
    # Acceleration plot
    ax2 = fig.add_subplot(142)
    ax2.plot(time_data, ax_data, 'r-', label='X')
    ax2.plot(time_data, ay_data, 'g-', label='Y')
    ax2.plot(time_data, az_data, 'b-', label='Z')
    ax2.set_title('Acceleration vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Acceleration (g)')
    ax2.grid(True)
    ax2.legend()
    
    # Velocity plot
    ax3 = fig.add_subplot(143)
    ax3.plot(time_data, vx_data, 'r-', label='X')
    ax3.plot(time_data, vy_data, 'g-', label='Y')
    ax3.plot(time_data, vz_data, 'b-', label='Z')
    ax3.set_title('Velocity vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.grid(True)
    ax3.legend()

    # Force plot
    ax4 = fig.add_subplot(144)
    ax4.plot(time_data, fx_data, 'r-', label='Force X (N)')
    ax4.plot(time_data, fy_data, 'g-', label='Force Y (N)')
    ax4.plot(time_data, fz_data, 'b-', label='Force Z (N)')
    ax4.set_title('Force vs Time')
    ax4.set_xlabel('Time (s)')
    ax4.set_ylabel('Force (N)')
    ax4.grid(True)
    ax4.legend()

    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Display the plot
    plt.show()
else:
    print("No data was collected to plot")
