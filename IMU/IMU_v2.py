import serial
import serial.tools.list_ports
import time
import numpy as np
import csv
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

########################################
# Helper function to set 3D axes to the same scale
########################################
def set_axes_equal_3d(ax):
    """
    Make the 3D axes of a Matplotlib figure have equal scale so that spheres 
    appear as spheres, cubes as cubes, etc.
    """
    x_limits = ax.get_xlim3d()
    y_limits = ax.get_ylim3d()
    z_limits = ax.get_zlim3d()

    x_range = abs(x_limits[1] - x_limits[0])
    x_middle = np.mean(x_limits)
    y_range = abs(y_limits[1] - y_limits[0])
    y_middle = np.mean(y_limits)
    z_range = abs(z_limits[1] - z_limits[0])
    z_middle = np.mean(z_limits)

    plot_radius = 0.5 * max([x_range, y_range, z_range])
    ax.set_xlim3d([x_middle - plot_radius, x_middle + plot_radius])
    ax.set_ylim3d([y_middle - plot_radius, y_middle + plot_radius])
    ax.set_zlim3d([z_middle - plot_radius, z_middle + plot_radius])

########################################
# 1. Kalman Filter Class (for acceleration smoothing)
########################################
class KalmanFilter:
    def __init__(self, process_variance, measurement_variance,
                 initial_value=0, initial_estimate_error=1):
        self.process_variance = process_variance
        self.measurement_variance = measurement_variance
        self.estimate = initial_value
        self.estimate_error = initial_estimate_error
        
    def update(self, measurement):
        # Prediction
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
        # Update
        kalman_gain = prediction_error / (prediction_error + self.measurement_variance)
        self.estimate = prediction + kalman_gain * (measurement - prediction)
        self.estimate_error = (1 - kalman_gain) * prediction_error
        return self.estimate

########################################
# 2. Helper to auto-detect Arduino port
########################################
def get_arduino_port():
    ports = list(serial.tools.list_ports.comports())
    for port in ports:
        if ("Arduino" in port.description or 
            "CH340" in port.description or 
            "USB Serial" in port.description):
            return port.device
    return None

########################################
# 3. Set up serial port
########################################
serial_port = get_arduino_port() or 'COM6'  # change if needed
baud_rate = 9600
print(f"Using port: {serial_port}")

try:
    ser = serial.Serial(serial_port, baud_rate, timeout=1)
    print(f"Connected to {serial_port}")
except serial.SerialException as e:
    print(f"Error opening port {serial_port}: {e}")
    exit(1)

########################################
# 4. Parameters and variables
########################################
duration = 10  # seconds to record data

# Create Kalman filters for each acceleration axis (smoothing the raw acceleration)
kf_ax = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_ay = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_az = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)

# Lists to store data for later plotting/export
time_data = []

# For velocity (result of integrating acceleration)
vel_x_data = []
vel_y_data = []
vel_z_data = []

# For position (result of integrating velocity)
pos_x_data = []
pos_y_data = []
pos_z_data = []

# Initialize variables:
# For calibration, we assume the sensor is static.
calib_ax = []
calib_ay = []
calib_az = []

# Velocity and position (in m/s and m, respectively)
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

position_x = 0.0
position_y = 0.0
position_z = 0.0

########################################
# 5. Calibration Step for Acceleration Bias
########################################
print("Calibrating acceleration bias for 2 seconds... Please keep the sensor still.")
calib_start = time.time()

while time.time() - calib_start < 2.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            # Expecting sensor to output: ax,ay,az (in m/s^2) as comma separated values
            raw_ax, raw_ay, raw_az = map(float, line.split(','))
            calib_ax.append(raw_ax)
            calib_ay.append(raw_ay)
            calib_az.append(raw_az)
        except ValueError:
            continue

if len(calib_ax) > 0:
    bias_ax = np.mean(calib_ax)
    bias_ay = np.mean(calib_ay)
    bias_az = np.mean(calib_az)
    # If the sensor is flat and its Z-axis is aligned with gravity,
    # subtract 9.81 m/s^2 so that static readings are zero.
    
    print(f"Calibration complete. Acceleration biases: ax={bias_ax:.3f}, ay={bias_ay:.3f}, az={bias_az:.3f}")
else:
    bias_ax, bias_ay, bias_az = 0, 0, 0
    print("No calibration data received; biases set to 0.")

########################################
# 6. Live Tracker Setup (3D Velocity Graph)
########################################
plt.ion()  # interactive mode on
fig_live = plt.figure(figsize=(8,6))
ax_live = fig_live.add_subplot(111, projection='3d')
line_live, = ax_live.plot([], [], [], 'b-', marker='o', label='Velocity Trajectory')
ax_live.set_xlabel('Vx (m/s)')
ax_live.set_ylabel('Vy (m/s)')
ax_live.set_zlabel('Vz (m/s)')
ax_live.set_title('Live 3D Velocity Tracker')
ax_live.legend()

# Set initial axis limits for live tracker (velocity scale)
ax_live.set_xlim(-1, 1)
ax_live.set_ylim(-1, 1)
ax_live.set_zlim(-1, 1)

plt.show()

########################################
# 7. Main Data Collection Loop Using Acceleration Data
########################################
print("Starting data collection using acceleration data...")
start_time = time.time()

while time.time() - start_time < duration:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        current_time = time.time() - start_time
        try:
            # Read sensor acceleration data (in m/s^2)
            raw_ax, raw_ay, raw_az = map(float, line.split(','))
            # Remove bias from acceleration
            meas_ax = raw_ax - bias_ax
            meas_ay = raw_ay - bias_ay
            meas_az = raw_az - bias_az

            # Apply Kalman filter for smoothing acceleration
            a_x = kf_ax.update(meas_ax)
            a_y = kf_ay.update(meas_ay)
            a_z = kf_az.update(meas_az)

            # Determine time step
            if len(time_data) > 0:
                dt = current_time - time_data[-1]
            else:
                dt = 0.01

            # Integrate acceleration to obtain velocity
            velocity_x += a_x * dt
            velocity_y += a_y * dt
            velocity_z += a_z * dt

            # Integrate velocity to obtain position
            position_x += velocity_x * dt
            position_y += velocity_y * dt
            position_z += velocity_z * dt

            # Save data for plotting/export
            time_data.append(current_time)
            vel_x_data.append(velocity_x)
            vel_y_data.append(velocity_y)
            vel_z_data.append(velocity_z)
            pos_x_data.append(position_x)
            pos_y_data.append(position_y)
            pos_z_data.append(position_z)

            # Update live velocity plot:
            line_live.set_data(vel_x_data, vel_y_data)
            line_live.set_3d_properties(vel_z_data)
            ax_live.relim()
            ax_live.autoscale_view()
            set_axes_equal_3d(ax_live)
            plt.draw()
            plt.pause(0.001)

            # Optional debug print
            print(f"t={current_time:.2f}s, dt={dt:.3f}, "
                  f"ax={a_x:.3f}, ay={a_y:.3f}, az={a_z:.3f}, "
                  f"Vx={velocity_x:.3f}, Vy={velocity_y:.3f}, Vz={velocity_z:.3f}, "
                  f"Px={position_x:.3f}, Py={position_y:.3f}, Pz={position_z:.3f}")
        except ValueError:
            continue

ser.close()
plt.ioff()
print("Serial port closed.")

########################################
# 8. Final Plots After Data Collection
########################################
# Figure 1: Velocity Plots (kept as is)
fig_final = plt.figure(figsize=(15,6))

# 3D Velocity Trajectory (Left subplot)
ax1 = fig_final.add_subplot(121, projection='3d')
ax1.plot(vel_x_data, vel_y_data, vel_z_data, 'b-', label='Velocity Trajectory')
ax1.scatter([vel_x_data[0]], [vel_y_data[0]], [vel_z_data[0]], color='g', s=100, label='Start')
ax1.scatter([vel_x_data[-1]], [vel_y_data[-1]], [vel_z_data[-1]], color='r', s=100, label='End')
ax1.set_xlabel('Vx (m/s)')
ax1.set_ylabel('Vy (m/s)')
ax1.set_zlabel('Vz (m/s)')
ax1.set_title('3D Velocity Trajectory')
ax1.legend()
ax1.grid(True)
set_axes_equal_3d(ax1)

# Velocity vs Time Plot (Right subplot)
ax2 = fig_final.add_subplot(122)
ax2.plot(time_data, vel_x_data, 'r', label='Vx')
ax2.plot(time_data, vel_y_data, 'g', label='Vy')
ax2.plot(time_data, vel_z_data, 'b', label='Vz')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Velocity vs Time')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()

# Figure 2: 3D Position Trajectory
fig_pos = plt.figure(figsize=(8,6))
ax_pos = fig_pos.add_subplot(111, projection='3d')
ax_pos.plot(pos_x_data, pos_y_data, pos_z_data, 'm-', label='Position Trajectory')
ax_pos.scatter([pos_x_data[0]], [pos_y_data[0]], [pos_z_data[0]], color='g', s=100, label='Start')
ax_pos.scatter([pos_x_data[-1]], [pos_y_data[-1]], [pos_z_data[-1]], color='r', s=100, label='End')
ax_pos.set_xlabel('X (m)')
ax_pos.set_ylabel('Y (m)')
ax_pos.set_zlabel('Z (m)')
ax_pos.set_title('3D Position Trajectory')
ax_pos.legend()
ax_pos.grid(True)
set_axes_equal_3d(ax_pos)
plt.show()

########################################
# 9. Export Data to CSV
########################################
csv_filename = "imu_data.csv"
with open(csv_filename, "w", newline="") as csvfile:
    writer = csv.writer(csvfile)
    # Write header (time, velocity, and position)
    writer.writerow(["time_s", "vel_x_m_s", "vel_y_m_s", "vel_z_m_s", "pos_x_m", "pos_y_m", "pos_z_m"])
    # Write data rows
    for i in range(len(time_data)):
        writer.writerow([
            time_data[i],
            vel_x_data[i],
            vel_y_data[i],
            vel_z_data[i],
            pos_x_data[i],
            pos_y_data[i],
            pos_z_data[i]
        ])

print(f"Data exported to {csv_filename}")
