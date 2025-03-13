import serial
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import time
import serial.tools.list_ports

def get_arduino_port():
    """Find the Arduino port automatically"""
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if "Arduino" in port.description or "CH340" in port.description or "USB Serial" in port.description:
            return port.device
    return None

serial_port = 'COM7'  # Specify the correct port
print(f"Using port: {serial_port}")
baud_rate = 9600  # Match this with your Arduino baud rate

try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Connected to {serial_port}")
except serial.SerialException as e:
    print(f"Error opening port {serial_port}: {e}")
    exit(1)

# Lists to store the accelerometer and computed data
time_data = []
ax_data = []
ay_data = []
az_data = []
vx = vy = vz = 0  # Initial velocities
x = y = z = 0  # Initial positions
vx_data = [vx]
vy_data = [vy]
vz_data = [vz]
x_data = [x]
y_data = [y]
z_data = [z]

duration = 5  # Record data for 5 seconds
start_time = time.time()

# Buffer to store the last three readings for Simpson's integration
ax_buffer = [0, 0, 0]
ay_buffer = [0, 0, 0]
az_buffer = [0, 0, 0]

try:
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            current_time = time.time()
            dt = current_time - start_time if len(time_data) == 0 else current_time - time_data[-1]
            line = ser.readline().decode('utf-8').strip()
            try:
                ax, ay, az = map(float, line.split(','))
                ax -= 0.04  # Calibration adjustment
                ay += 0.08
                az += 0.97

                ax_buffer.pop(0)
                ay_buffer.pop(0)
                az_buffer.pop(0)
                ax_buffer.append(ax)
                ay_buffer.append(ay)
                az_buffer.append(az)
                
                if len(time_data) >= 2:  # Ensure we have at least three data points
                    # Simpson's Rule Integration
                    vx += (dt/3) * (ax_buffer[0] + 4*ax_buffer[1] + ax_buffer[2])
                    vy += (dt/3) * (ay_buffer[0] + 4*ay_buffer[1] + ay_buffer[2])
                    vz += (dt/3) * (az_buffer[0] + 4*az_buffer[1] + az_buffer[2])

                    # Update position
                    x += (dt/3) * (vx_data[-2] + 4*vx_data[-1] + vx)
                    y += (dt/3) * (vy_data[-2] + 4*vy_data[-1] + vy)
                    z += (dt/3) * (vz_data[-2] + 4*vz_data[-1] + vz)

                # Store the time and data
                time_data.append(current_time)
                ax_data.append(ax)
                ay_data.append(ay)
                az_data.append(az)
                vx_data.append(vx)
                vy_data.append(vy)
                vz_data.append(vz)
                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

            except ValueError as e:
                print(f"Invalid data received: {line}")
except KeyboardInterrupt:
    print("\nData collection stopped by user")
finally:
    ser.close()
    print("Serial connection closed")

# Plotting the results
if time_data:
    fig = plt.figure(figsize=(15, 5))
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(x_data, y_data, z_data, 'b-', label='Position')
    ax1.set_title('3D Position Trajectory')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Z Position (m)')
    ax1.grid(True)

    ax2 = fig.add_subplot(132)
    ax2.plot(time_data, ax_data, 'r-', label='X')
    ax2.plot(time_data, ay_data, 'g-', label='Y')
    ax2.plot(time_data, az_data, 'b-', label='Z')
    ax2.set_title('Acceleration vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Acceleration (g)')
    ax2.grid(True)
    ax2.legend()

    ax3 = fig.add_subplot(133)
    ax3.plot(time_data, vx_data, 'r-', label='X')
    ax3.plot(time_data, vy_data, 'g-', label='Y')
    ax3.plot(time_data, vz_data, 'b-', label='Z')
    ax3.set_title('Velocity vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.grid(True)
    ax3.legend()

    plt.tight_layout()
    plt.show()
else:
    print("No data was collected to plot")
