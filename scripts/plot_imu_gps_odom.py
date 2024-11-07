#import rosbag2_py
# Function to read and extract data from the bag file
import rosbag
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import numpy as np
#from rclpy.serialization import deserialize_message
from sensor_msgs.msg import NavSatFix, Imu
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
from scipy.spatial.transform import Rotation as R
from pyproj import Proj

# Assuming the transformation matrices are available
# Example 4x4 transformation matrices
T_imu_to_odom = np.array([
    [0.5402, -0.8414, -0.0144, 0],   # X-axis transformation remains the same
    [0.8414, 0.5402, -0.0013, 0],  # Y-axis transformation remains the same
    [0.0067, 0.0128, 1, 0],  # Z-axis is inverted (flip sign)
    [0, 0, 0, 1]
])


T_gps_to_odom = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_gps_to_odom = np.array([
    [-0.5402, -0.8414, 0, 0],
    [0.8414, -0.5402, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])
T_gps_to_odom = np.array([
    [-0.8414, -0.5403, 0, 0],
    [0.5403, -0.8414, 0, 0],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
])


# Function to transform IMU/GPS data
def transform_data(data, T):
    """ Apply 4x4 transformation to the input data (IMU or GPS) """
    # Convert to homogeneous coordinates by adding a 1 as the fourth element
    homogeneous_data = np.hstack([data, np.ones((data.shape[0], 1))])
    # Apply the transformation matrix
    transformed_data = (T @ homogeneous_data.T).T
    # Return the transformed data (drop the homogeneous coordinate)
    return transformed_data[:, :3]

# WGS84 ellipsoid parameters
a = 6378137.0  # semi-major axis in meters
b = 6356752.314245  # semi-minor axis in meters
e2 = 1 - (b**2 / a**2)  # eccentricity squared

def geodetic_to_ecef(lat, lon, alt):
    """Convert geodetic coordinates to ECEF."""
    lat, lon = np.radians(lat), np.radians(lon)
    N = a / np.sqrt(1 - e2 * np.sin(lat) ** 2)
    x = (N + alt) * np.cos(lat) * np.cos(lon)
    y = (N + alt) * np.cos(lat) * np.sin(lon)
    z = (N * (b**2 / a**2) + alt) * np.sin(lat)
    return np.array([x, y, z])

def ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon):
    """Convert ECEF to ENU coordinates."""
    ref_lat, ref_lon = np.radians(ref_lat), np.radians(ref_lon)
    dx, dy, dz = ecef - ref_ecef

    # Rotation matrix for ECEF to ENU
    t = np.cos(ref_lon) * dx + np.sin(ref_lon) * dy
    east = -np.sin(ref_lon) * dx + np.cos(ref_lon) * dy
    north = -np.sin(ref_lat) * t + np.cos(ref_lat) * dz
    up = np.cos(ref_lat) * t + np.sin(ref_lat) * dz
    return np.array([east, north, up])

def gps_to_xy(gps_data, ref_lat, ref_lon, ref_alt=0):
    """Convert a list of GPS data to ENU coordinates."""
    ref_ecef = geodetic_to_ecef(ref_lat, ref_lon, ref_alt)
    positions = []

    for gps in gps_data:
        time = gps[0]
        lat, lon, alt = gps[1], gps[2], gps[3] if len(gps) > 3 else 0
        ecef = geodetic_to_ecef(lat, lon, alt)
        enu = ecef_to_enu(ecef, ref_ecef, ref_lat, ref_lon)
        positions.append([time, enu[0], enu[1]])

    return np.array(positions)

# Integrate IMU data to get the position over time
def integrate_imu(imu_data):
    positions = []
    velocity = np.zeros(2)  # Initialize velocity in 2D
    position = np.array([0, 0])  # Initialize position in 2D
    orientation_z = 0.0  # Yaw

    prev_time = imu_data[0][0]  # Get the first timestamp

    for i, imu in enumerate(imu_data):
        # Get time, linear acceleration, and angular velocity
        time = imu[0]
        accel = imu[1:3]  # Extract (x, y) linear acceleration
        ang_vel_z = imu[9]  # Z-axis angular velocity (yaw rate)

        dt = time - prev_time
        prev_time = time

        # Update orientation (yaw)
        orientation_z += ang_vel_z * dt
        R_matrix = np.array([[np.cos(orientation_z), -np.sin(orientation_z)],
                             [np.sin(orientation_z), np.cos(orientation_z)]])
        # Rotate acceleration from IMU frame to world frame
        accel_world = R_matrix @ accel
        #print(accel)
        # Update velocity and position
        velocity += accel_world * dt
        #print(velocity)
        position = position.astype('float64')
        position += velocity * dt

        # Apply transformation from base (odometry) frame to world frame AFTER position is updated
        position_homogeneous = np.hstack([position, 0, 1])  # Make it a homogeneous coordinate (x, y, 0, 1)
        position_transformed = (T_imu_to_odom @ position_homogeneous)[:2]  # Apply transformation and extract (x, y)

        # Store the 2D transformed position
        positions.append([time, position_transformed[0], position_transformed[1]])

    return np.array(positions)

# Function to read the ROS bag and extract odometry, IMU, and GPS data
def read_bag(bag_path):
    bag = rosbag.Bag(bag_path)
    
    odometry_data = []
    imu_data = []
    gps_data = []

    for topic, msg, t in bag.read_messages(topics=['/wamv/sensors/position/ground_truth_odometry',
                                                   '/wamv/sensors/imu/imu/data',
                                                   '/wamv/sensors/gps/gps/fix']):
        # Handling the ground truth odometry message (nav_msgs/Odometry)
        if topic == "/wamv/sensors/position/ground_truth_odometry":
            odometry_data.append((msg.header.stamp.to_sec(),
                                  msg.pose.pose.position.x+532.64, msg.pose.pose.position.y-161.91))

        # Handling the IMU data (sensor_msgs/Imu)
        elif topic == "/wamv/sensors/imu/imu/data":
            imu_data.append((msg.header.stamp.to_sec(),
                             msg.linear_acceleration.x, msg.linear_acceleration.y,
                             msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w,
                             msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z))

        # Handling the GPS data (sensor_msgs/NavSatFix)
        elif topic == "/wamv/sensors/gps/gps/fix":
            gps_data.append((msg.header.stamp.to_sec(),
                             msg.latitude, msg.longitude))

    bag.close()

    return np.array(odometry_data), np.array(imu_data), np.array(gps_data)

# Plot the data without interpolation and RMSE calculations
def plot_data(odometry_data, imu_data, gps_data):
    # Convert GPS data to (x, y) coordinates
    ref_lat, ref_lon = gps_data[0][1], gps_data[0][2]  # Reference latitude and longitude
    gps_xy = gps_to_xy(gps_data, ref_lat, ref_lon)
    
    # Integrate IMU data to get position
    #imu_positions = integrate_imu(imu_data)
    
    # Plotting the positions
    plt.figure(figsize=(10, 6))
    
    # Odometry Ground Truth
    plt.plot(odometry_data[:, 1], odometry_data[:, 2], label='Odometry (Ground Truth)', color='green', linewidth=2.5)

    # GPS Derived Position
    plt.plot(gps_xy[:, 1], gps_xy[:, 2], label='GPS Derived', color='blue', linewidth=2.5)

    # IMU Derived Position
    #plt.plot(imu_positions[:, 1], imu_positions[:, 2], label='IMU Derived', color='red', linewidth=2.5)

    # Set axis labels and title
    plt.xlabel('X Position', fontsize=18)
    plt.ylabel('Y Position', fontsize=18)
    plt.title('Odometry, GPS, and IMU Positions', fontsize=18)

    # Ensure equal scaling for both axes
    plt.axis('equal')

    # Add a legend, grid, and adjust tick sizes
    plt.legend(fontsize=18)
    plt.tick_params(axis='both', which='major', labelsize=18)
    plt.grid(True)

    # Show the plot
    plt.show()

# Example usage
if __name__ == "__main__":
    bag_path = 'output.bag'  # Replace with the path to your ROS2 bag file
    odometry_data, imu_data, gps_data = read_bag(bag_path)
    plot_data(odometry_data, imu_data, gps_data)

