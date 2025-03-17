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
        prediction = self.estimate
        prediction_error = self.estimate_error + self.process_variance
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
duration = 15  # seconds to record data

# Create Kalman filters for each acceleration axis
kf_ax = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_ay = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_az = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)

# Lists for plotting/exporting
time_data = []
vel_x_data = []
vel_y_data = []
vel_z_data = []
pos_x_data = []
pos_y_data = []
pos_z_data = []
punch_labels = []  # 1 if in punch, 0 otherwise
punch_type_labels = []  # NEW: List to store punch types

# Initial variables for velocity and position
velocity_x = 0.0
velocity_y = 0.0
velocity_z = 0.0

position_x = 0.0
position_y = 0.0
position_z = 0.0

# Calibration variables
calib_ax = []
calib_ay = []
calib_az = []

# Punch segmentation thresholds (adjust as needed)
punch_start_threshold = 2.0  # m/s²
punch_end_threshold = 1.0    # m/s²
in_punch = False  # state flag

########################################
# 5. Calibration Step for Acceleration Bias
########################################
print("Calibrating acceleration bias for 2 seconds... Please keep the sensor still.")
calib_start = time.time()

while time.time() - calib_start < 2.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            raw_ax, raw_ay, raw_az = map(float, line.split(','))
            # Convert from g to m/s²
            raw_ax *= 9.81  
            raw_ay *= 9.81
            raw_az *= 9.81
            calib_ax.append(raw_ax)
            calib_ay.append(raw_ay)
            calib_az.append(raw_az)
        except ValueError:
            continue

if len(calib_ax) > 0:
    bias_ax = np.mean(calib_ax)
    bias_ay = np.mean(calib_ay)
    bias_az = np.mean(calib_az)
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
            # Read and convert sensor acceleration (g to m/s²)
            raw_ax, raw_ay, raw_az = map(float, line.split(','))
            raw_ax *= 9.81  
            raw_ay *= 9.81
            raw_az *= 9.81
            
            # Remove bias
            meas_ax = raw_ax - bias_ax
            meas_ay = raw_ay - bias_ay
            meas_az = raw_az - bias_az

            # Apply Kalman filter
            a_x = kf_ax.update(meas_ax)
            a_y = kf_ay.update(meas_ay)
            a_z = kf_az.update(meas_az)

            # Compute overall acceleration magnitude
            acc_magnitude = np.sqrt(a_x**2 + a_y**2 + a_z**2)

            # Update punch segmentation state
            if not in_punch and acc_magnitude > punch_start_threshold:
                in_punch = True
                print("Punch started!")
                punch_labels.append(1)
                punch_type_labels.append("")  # Empty string while punch is ongoing
            elif in_punch and acc_magnitude < punch_end_threshold:
                in_punch = False
                # Mark the last entry as "jab" when punch ends
                if punch_type_labels:
                    punch_type_labels[-1] = "jab"  # Label the last point of the punch
                print("Punch ended, velocity reset.")
                punch_labels.append(0)
                punch_type_labels.append("")
            else:
                punch_labels.append(1 if in_punch else 0)
                punch_type_labels.append("")

            # Determine dt (time step)
            if len(time_data) > 0:
                dt = current_time - time_data[-1]
            else:
                dt = 0.01

            # Only integrate if in punch; otherwise, hold velocity at zero.
            if in_punch:
                velocity_x += a_x * dt
                velocity_y += a_y * dt
                velocity_z += a_z * dt
            else:
                # Ensure velocity remains zero when no punch is detected.
                velocity_x, velocity_y, velocity_z = 0.0, 0.0, 0.0

            # Integrate velocity to update position
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

            # Update live plot
            line_live.set_data(vel_x_data, vel_y_data)
            line_live.set_3d_properties(vel_z_data)
            ax_live.relim()
            ax_live.autoscale_view()
            set_axes_equal_3d(ax_live)
            plt.draw()
            plt.pause(0.001)

            # Debug print
            print(f"t={current_time:.2f}s, dt={dt:.3f}, "
                  f"ax={a_x:.3f}, ay={a_y:.3f}, az={a_z:.3f}, "
                  f"acc_mag={acc_magnitude:.3f}, "
                  f"Vx={velocity_x:.3f}, Vy={velocity_y:.3f}, Vz={velocity_z:.3f}, "
                  f"Punch={'Yes' if in_punch else 'No'}")
        except ValueError:
            continue

ser.close()
plt.ioff()
print("Serial port closed.")

########################################
# 8. Final Plots After Data Collection
########################################
# Figure 1: Velocity Plots
fig_final = plt.figure(figsize=(15,6))
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
    writer.writerow([
        "time_s", 
        "acc_x_m_s2", "acc_y_m_s2", "acc_z_m_s2",
        "vel_x_m_s", "vel_y_m_s", "vel_z_m_s", 
        "pos_x_m", "pos_y_m", "pos_z_m",
        "punch",
        "punch_type"  # New column
    ])
    for i in range(len(time_data)):
        writer.writerow([
            time_data[i],
            kf_ax.update(raw_ax - bias_ax),
            kf_ay.update(raw_ay - bias_ay),
            kf_az.update(raw_az - bias_az),
            vel_x_data[i],
            vel_y_data[i],
            vel_z_data[i],
            pos_x_data[i],
            pos_y_data[i],
            pos_z_data[i],
            punch_labels[i],
            punch_type_labels[i]  # Add punch type to CSV
        ])

print(f"Data exporte to {csv_filename}")
