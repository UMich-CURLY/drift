import matplotlib.pyplot as plt

# Initialize lists for storing parsed data
time_estimated, time_measured, time_ground_truth = [], [], []
ground_truth_x, ground_truth_y, ground_truth_z = [], [], []
gps_measured_x, gps_measured_y, gps_measured_z = [], [], []
estimated_x, estimated_y, estimated_z = [], [], []

# Parse 'debugging_data.txt' for estimated and GPS measured values
with open('debugging_data.txt', 'r') as file:
    for line in file:
        # Extract and parse Filter and Measurement data
        filter_time = float(line.split("Filter time: ")[1].split(" ")[0])
        filter_state = line.split("Filter state: ")[1].split(";")[0]
        meas_time = float(line.split("Measurement time: ")[1].split(" ")[0])
        meas_state = line.split("Measurement state: ")[1].strip()

        # Parse coordinates for Filter (estimated) and GPS (measured) states
        fx, fy, fz = map(float, filter_state.split())
        mx, my, mz = map(float, meas_state.split())

        # Append data
        time_estimated.append(filter_time)
        time_measured.append(meas_time)
        estimated_x.append(fx)
        estimated_y.append(fy)
        estimated_z.append(fz)
        gps_measured_x.append(mx)
        gps_measured_y.append(my)
        gps_measured_z.append(mz)

# Parse 'debugging_data_gtodom.txt' for ground truth values
with open('debugging_data_gtodom.txt', 'r') as file:
    for line in file:
        # Extract and parse Measurement time and state
        meas_time = float(line.split("Measurement time: ")[1].split(" ")[0])
        meas_state = line.split("Measurement state: ")[1].strip()
        
        # Parse coordinates for ground truth
        gx, gy, gz = map(float, meas_state.split())

        # Append ground truth data
        time_ground_truth.append(meas_time)
        ground_truth_x.append(gx)
        ground_truth_y.append(gy)
        ground_truth_z.append(gz)

# Normalize times to start from 0 (using the earliest timestamp across all)
start_time = min(time_estimated[0], time_measured[0], time_ground_truth[0])
time_estimated = [(t - start_time) for t in time_estimated]
time_measured = [(t - start_time) for t in time_measured]
time_ground_truth = [(t - start_time) for t in time_ground_truth]

# Plot x vs time
plt.figure(figsize=(15, 5))
plt.plot(time_estimated, estimated_x, label='DRIFT Corrected')
plt.plot(time_measured, gps_measured_x, label='GPS Measured')
plt.plot(time_ground_truth, ground_truth_x, label='Ground Truth')
plt.xlabel('Time (s)')
plt.ylabel('X Coordinate')
plt.title('X vs Time')
plt.legend()
plt.grid(True)
plt.show()

# Plot y vs time
plt.figure(figsize=(15, 5))
plt.plot(time_estimated, estimated_y, label='DRIFT Corrected')
plt.plot(time_measured, gps_measured_y, label='GPS Measured')
plt.plot(time_ground_truth, ground_truth_y, label='Ground Truth')
plt.xlabel('Time (s)')
plt.ylabel('Y Coordinate')
plt.title('Y vs Time')
plt.legend()
plt.grid(True)
plt.show()

# Plot z vs time
plt.figure(figsize=(15, 5))
plt.plot(time_estimated, estimated_z, label='DRIFT Corrected')
plt.plot(time_measured, gps_measured_z, label='GPS Measured')
plt.plot(time_ground_truth, ground_truth_z, label='Ground Truth')
plt.xlabel('Time (s)')
plt.ylabel('Z Coordinate')
plt.title('Z vs Time')
plt.legend()
plt.grid(True)
plt.show()

