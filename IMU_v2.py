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

# Use COM5 directly since we know that's our port
serial_port = 'COM7'
print(f"Using port: {serial_port}")

baud_rate = 9600  # Match this with your Arduino baud rate

try:
    ser = serial.Serial(serial_port, baud_rate)
    print(f"Connected to {serial_port}")
except serial.SerialException as e:
    print(f"Error opening port {serial_port}: {e}")
    exit(1)

# Lists to store the accelerometer data
time_data = []
ax_data = []
ay_data = []
az_data = []
vx_data = []
vy_data = []
vz_data = []
x_data = []
y_data = []
z_data = []
vx = 0
vy = 0
vz = 0
x = 0
y = 0
z = 0

# Capture data for a certain amount of time (in seconds)
duration = 5  # Record data for 10 seconds
start_time = time.time()

try:
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            # Read one line of data from the serial port
            line = ser.readline().decode('utf-8').strip()  # Decode byte to string and strip newline
            try:
                # Split the line into the three acceleration values (Ax, Ay, Az)
                ax, ay, az = map(float, line.split(','))
                # Store the data
                ax = ax - 0.04
                ay = ay + 0.08
                az = az + 0.97
                time_data.append(time.time() - start_time)
                ax_data.append(ax)
                ay_data.append(ay)
                az_data.append(az)
                print(f"Time: {time_data[-1]:.2f}s, Ax: {ax:.2f}, Ay: {ay:.2f}, Az: {az:.2f}")

                dt = 1/(9600/3)

                vx = vx + ax * dt
                vy = vy + ay * dt
                vz = vz + az * dt

                vx_data.append(vx)
                vy_data.append(vy)
                vz_data.append(vz)

                x = x + vx * dt
                y = y + vy * dt
                z = z + vz * dt

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
    fig = plt.figure(figsize=(15, 5))
    
    # 3D position plot
    ax1 = fig.add_subplot(131, projection='3d')
    ax1.plot(x_data, y_data, z_data, 'b-', label='Position')
    ax1.set_title('3D Position Trajectory')
    ax1.set_xlabel('X Position (m)')
    ax1.set_ylabel('Y Position (m)')
    ax1.set_zlabel('Z Position (m)')
    ax1.grid(True)
    

    # ax1.set_xlim(-1, 1)
    # ax1.set_ylim(-1, 1)
    # ax1.set_zlim(-1, 1)
    
    # Acceleration plot
    ax2 = fig.add_subplot(132)
    ax2.plot(time_data, ax_data, 'r-', label='X')
    ax2.plot(time_data, ay_data, 'g-', label='Y')
    ax2.plot(time_data, az_data, 'b-', label='Z')
    ax2.set_title('Acceleration vs Time')
    ax2.set_xlabel('Time (s)')
    ax2.set_ylabel('Acceleration (g)')
    ax2.grid(True)
    ax2.legend()
    
    # Velocity plot
    ax3 = fig.add_subplot(133)
    ax3.plot(time_data, vx_data, 'r-', label='X')
    ax3.plot(time_data, vy_data, 'g-', label='Y')
    ax3.plot(time_data, vz_data, 'b-', label='Z')
    ax3.set_title('Velocity vs Time')
    ax3.set_xlabel('Time (s)')
    ax3.set_ylabel('Velocity (m/s)')
    ax3.grid(True)
    ax3.legend()
    
    # Adjust layout to prevent overlap
    plt.tight_layout()
    
    # Display the plot
    plt.show()
else:
    print("No data was collected to plot")