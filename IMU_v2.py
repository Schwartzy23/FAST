import serial
import serial.tools.list_ports
import time
import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D

########################################
# 1. Kalman Filter Class (for velocity smoothing)
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

# We'll create Kalman filters for each velocity axis (optional smoothing)
kf_vx = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_vy = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)
kf_vz = KalmanFilter(process_variance=0.1, measurement_variance=0.1, initial_value=0)

# Lists to store data for later plotting
time_data = []
vx_data = []
vy_data = []
vz_data = []
x_data = []
y_data = []
z_data = []

# Position variables (in meters)
x = 0.0
y = 0.0
z = 0.0

########################################
# 5. Calibration Step for Velocity Bias
########################################
# Assume sensor is still for 2 seconds so that measured velocity should be zero.
print("Calibrating velocity bias for 2 seconds... Please keep the sensor still.")
calib_start = time.time()
calib_vx = []
calib_vy = []
calib_vz = []

while time.time() - calib_start < 2.0:
    if ser.in_waiting > 0:
        line = ser.readline().decode('utf-8').strip()
        try:
            # Expecting sensor to output: vx,vy,vz (in m/s) as comma separated values
            raw_vx, raw_vy, raw_vz = map(float, line.split(','))
            calib_vx.append(raw_vx)
            calib_vy.append(raw_vy)
            calib_vz.append(raw_vz)
        except ValueError:
            continue

if len(calib_vx) > 0:
    bias_vx = np.mean(calib_vx)
    bias_vy = np.mean(calib_vy)
    bias_vz = np.mean(calib_vz)
    print(f"Calibration complete. Velocity biases: vx={bias_vx:.3f}, vy={bias_vy:.3f}, vz={bias_vz:.3f}")
else:
    bias_vx, bias_vy, bias_vz = 0, 0, 0
    print("No calibration data received; biases set to 0.")

########################################
# 6. Live Tracker Setup (3D Plot) with Larger Scales
########################################
plt.ion()  # interactive mode on
fig_live = plt.figure(figsize=(8,6))
ax_live = fig_live.add_subplot(111, projection='3d')
line_live, = ax_live.plot([], [], [], 'b-', marker='o', label='Trajectory')
ax_live.set_xlabel('X (m)')
ax_live.set_ylabel('Y (m)')
ax_live.set_zlabel('Z (m)')
ax_live.set_title('Live Velocity-Based Position Tracker')
ax_live.legend()
# Set larger limits for live tracking:
ax_live.set_xlim(-1, 1)
ax_live.set_ylim(-1, 1)
ax_live.set_zlim(-1, 1)
plt.show()

########################################
# 7. Main Data Collection Loop Using Velocity Vectors
########################################
print("Starting data collection using velocity vectors...")
start_time = time.time()
first_data = True

try:
    while time.time() - start_time < duration:
        if ser.in_waiting > 0:
            line = ser.readline().decode('utf-8').strip()
            current_time = time.time() - start_time
            try:
                # Read sensor velocity data (in m/s)
                raw_vx, raw_vy, raw_vz = map(float, line.split(','))
                # Remove bias
                meas_vx = raw_vx - bias_vx
                meas_vy = raw_vy - bias_vy
                meas_vz = raw_vz - bias_vz

                # Optional: Apply Kalman filter for smoothing
                vx = kf_vx.update(meas_vx)
                vy = kf_vy.update(meas_vy)
                vz = kf_vz.update(meas_vz)

                # Determine time step
                if len(time_data) > 0:
                    dt = current_time - time_data[-1]
                else:
                    dt = 0.01

                # Update position using velocity vectors directly:
                x += vx * dt
                y += vy * dt
                z += vz * dt

                # Save data for plotting
                time_data.append(current_time)
                vx_data.append(vx)
                vy_data.append(vy)
                vz_data.append(vz)
                x_data.append(x)
                y_data.append(y)
                z_data.append(z)

                # Update live plot:
                line_live.set_data(x_data, y_data)
                line_live.set_3d_properties(z_data)
                ax_live.relim()
                ax_live.autoscale_view()
                plt.draw()
                plt.pause(0.001)

                # Optional debug print
                print(f"t={current_time:.2f}s, dt={dt:.3f}, "
                      f"vx={vx:.3f}, vy={vy:.3f}, vz={vz:.3f}, "
                      f"x={x:.3f}, y={y:.3f}, z={z:.3f}")
            except ValueError:
                continue

except KeyboardInterrupt:
    print("\nData collection stopped by user.")
finally:
    ser.close()
    plt.ioff()
    print("Serial port closed.")

########################################
########################################
# 8. Final Plots After Data Collection
########################################
fig_final = plt.figure(figsize=(15,6))

# 3D Trajectory Plot (Left subplot)
ax1 = fig_final.add_subplot(121, projection='3d')
ax1.plot(x_data, y_data, z_data, 'b-', label='Trajectory')
ax1.scatter([x_data[0]], [y_data[0]], [z_data[0]], color='g', s=100, label='Start')
ax1.scatter([x_data[-1]], [y_data[-1]], [z_data[-1]], color='r', s=100, label='End')
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_zlabel('Z (m)')
ax1.set_title('3D Position Trajectory')
ax1.legend()
ax1.grid(True)

# Velocity vs Time Plot (Right subplot)
ax2 = fig_final.add_subplot(122)
ax2.plot(time_data, vx_data, 'r', label='Vx')
ax2.plot(time_data, vy_data, 'g', label='Vy')
ax2.plot(time_data, vz_data, 'b', label='Vz')
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Velocity (m/s)')
ax2.set_title('Velocity Vectors vs Time')
ax2.legend()
ax2.grid(True)

plt.tight_layout()
plt.show()