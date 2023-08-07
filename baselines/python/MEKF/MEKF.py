'''
/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   MEKF.py
 *  @author Wenzhe Tong
 *  @brief  Multiplicative EKF
 *  @date   August 4, 2023
 **/
'''

import rosbag
import numpy as np
import time
import matplotlib.pyplot as plt
from tqdm import tqdm
from scipy.linalg import expm

def skew_symmetric(v):
    """Skew symmetric form of a 3x1 vector."""
    return np.array(
        [[0, -v[2], v[1]],
         [v[2], 0, -v[0]],
         [-v[1], v[0], 0]], dtype=np.float64)

def rot2quat(rot_matrix):
    # Ensure the input is a 3x3 numpy array
    rot_matrix = np.array(rot_matrix)
    if rot_matrix.shape != (3, 3):
        raise ValueError("Input matrix must be a 3x3 numpy array.")

    # Extract the elements of the rotation matrix
    m11, m12, m13 = rot_matrix[0, 0], rot_matrix[0, 1], rot_matrix[0, 2]
    m21, m22, m23 = rot_matrix[1, 0], rot_matrix[1, 1], rot_matrix[1, 2]
    m31, m32, m33 = rot_matrix[2, 0], rot_matrix[2, 1], rot_matrix[2, 2]

    # Calculate the components of the quaternion
    w = np.sqrt(max(0, 1 + m11 + m22 + m33)) / 2
    x = np.sqrt(max(0, 1 + m11 - m22 - m33)) / 2
    y = np.sqrt(max(0, 1 - m11 + m22 - m33)) / 2
    z = np.sqrt(max(0, 1 - m11 - m22 + m33)) / 2

    # Determine the signs of the quaternion components
    x = np.copysign(x, m32 - m23)
    y = np.copysign(y, m13 - m31)
    z = np.copysign(z, m21 - m12)

    # Assemble the quaternion
    quaternion = np.array([w, x, y, z])
    return quaternion

def quat2rot(quaternion):
    # Ensure the input is a 4-element numpy array or list
    if type(quaternion) == list:
        quaternion = np.array(quaternion)
    elif type(quaternion) == np.ndarray and (quaternion.shape == (1, 4) or quaternion.shape == (4, 1)):
        quaternion = quaternion.flatten()

    if quaternion.shape != (4,):
        raise ValueError("Input quaternion must be a 4-element numpy array or list.")

    # Normalize the quaternion to ensure it is a unit quaternion
    quaternion /= np.linalg.norm(quaternion)

    w, x, y, z = quaternion

    # Calculate the elements of the rotation matrix
    m11 = 1 - 2*y**2 - 2*z**2
    m12 = 2*x*y - 2*w*z
    m13 = 2*x*z + 2*w*y

    m21 = 2*x*y + 2*w*z
    m22 = 1 - 2*x**2 - 2*z**2
    m23 = 2*y*z - 2*w*x

    m31 = 2*x*z - 2*w*y
    m32 = 2*y*z + 2*w*x
    m33 = 1 - 2*x**2 - 2*y**2

    # Assemble the rotation matrix
    rotation_matrix = np.array([[m11, m12, m13],
                                [m21, m22, m23],
                                [m31, m32, m33]])

    return rotation_matrix


def read_imu(bag, topic_name):
    imu_data = []
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        imu_data.append([t.to_sec(), msg.angular_velocity.x, msg.angular_velocity.y, msg.angular_velocity.z,
                         msg.linear_acceleration.x, msg.linear_acceleration.y, msg.linear_acceleration.z,
                         msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w])
    imu_data = np.array(imu_data)
    freq = (imu_data.shape[0] - 1) / (imu_data[-1, 0] - imu_data[0, 0])

    print('imu data shape: ', imu_data.shape)
    print('imu data frequency: ', freq)
    return imu_data

def read_vel(bag, topic_name):
    vel_data = []
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        vel_data.append([t.to_sec(), msg.twist.twist.linear.x, 0, 0])
    vel_data = np.array(vel_data)
    freq = (vel_data.shape[0] - 1) / (vel_data[-1, 0] - vel_data[0, 0])
    print('vel data shape: ', vel_data.shape)
    print('vel data frequency: ', freq)
    return vel_data

def read_husky(bag, topic_name):
    vel_data = []
    radius = 0.158
    for topic, msg, t in bag.read_messages(topics=[topic_name]):
        v = (msg.velocity[0] + msg.velocity[1] + msg.velocity[2] + msg.velocity[3]) / 4. * radius

        vel_data.append([t.to_sec(), v, 0, 0])
    vel_data = np.array(vel_data)
    freq = (vel_data.shape[0] - 1) / (vel_data[-1, 0] - vel_data[0, 0])
    print('vel data shape: ', vel_data.shape)
    print('vel data frequency: ', freq)
    return vel_data

def init_bias(imu_data, R_imu2body, window_size=250):
    gyro_bias = R_imu2body@(np.mean(imu_data[:window_size, 1:4], axis=0)).T
    accel_bias = R_imu2body@(np.mean(imu_data[:window_size, 4:7], axis=0)-np.array([0, 0, 9.8]).reshape(3)).T
    return gyro_bias, accel_bias

def save_pose_to_tum(t, p, q, filename='tum_traj.txt'):
    # t = data[:, 0]
    # x = data[:, 1]
    # y = data[:, 2]
    # z = data[:, 3]
    # qw = np.ones(t.shape[0])
    # qx = np.zeros(t.shape[0])
    # qy = np.zeros(t.shape[0])
    # qz = np.zeros(t.shape[0])
    # t = data[:, 0]
    # x = data[:, 1]
    # y = data[:, 2]
    # z = data[:, 3]
    # qw = np.ones(t.shape[0])
    # qx = np.zeros(t.shape[0])
    # qy = np.zeros(t.shape[0])
    # qz = np.zeros(t.shape[0])
    s = 1
    if t.shape[0] > 15000:
        s = 5
    t = t[::s]
    x = p[::s, 0]
    y = p[::s, 1]
    z = p[::s, 2]
    qw = q[::s, 0]
    qx = q[::s, 1]
    qy = q[::s, 2]
    qz = q[::s, 3]
    # print(t.shape, x.shape, y.shape, z.shape, qw.shape, qx.shape, qy.shape, qz.shape)
    tum_data = np.stack((t, x, y, z, qx, qy, qz, qw), axis=1)

    # save to txt file
    np.savetxt(filename, tum_data, delimiter=' ')

def propoagate(t_imu, acc_data, gyro_data, acc_bias, gyro_bias, p_est, v_est, q_est, g, R_imu2body):
    g = np.array([0, 0, 9.81]).reshape(3)
    dt = t_imu[k] - t_imu[k-1]

    R_ns = quat2rot(q_est[k-1, :])
    f_ns = (R_ns @ (R_imu2body @ acc_data[k]-acc_bias).reshape((3,1))) - g.reshape((3,1)) # acc_bias in body frame
    assert f_ns.shape == (3,1)

    p_ = p_est[k-1, :].reshape((3,1)) + (v_est[k-1, :]*dt).reshape((3,1)) + 0.5*f_ns*dt**2 
    v_ = v_est[k-1, :].reshape((3,1)) + f_ns*dt
    # q_ = q_prev.quat_mult_left(q_curr).reshape((4,1))
    R_predict = R_ns @ expm(skew_symmetric((R_imu2body@gyro_data[k]-gyro_bias)*dt))
    # print(gyro_data[k])
    # print("---")
    q_ = rot2quat(R_predict).reshape((4,1))

    zero = np.zeros((3,3))
    I = np.eye(3)
    F = expm( np.block([[ zero ,   I,                                            zero],
                        [ zero ,   zero, -R_predict @ skew_symmetric(R_imu2body@acc_data[k] - acc_bias)],
                        [ zero ,   zero,      -skew_symmetric(R_imu2body@gyro_data[k]-gyro_bias)]])*dt )

    # 1.2 uncertainty propagation
    q_cov = np.zeros([9, 9]) # IMU noise covariance
    q_cov[0:3, 0:3] = 0.000001*np.eye(3)
    q_cov[3:6, 3:6] = acc_cov * np.eye(3) *dt**2
    q_cov[6:9, 6:9] = gyro_cov * np.eye(3) *dt**2

    l_jac = np.zeros((9,6))
    l_jac[3:, :] = np.eye(6) # motion model noise jacobian
    p_cov_ = F @ p_cov[k-1, :, :] @ F.T + q_cov 

    return p_, v_, q_, p_cov_

def measurement_update(sensor_var, y, p_cov, p, v, q):
    assert y.shape == (3,1)
    assert p.shape == (3,1)
    assert v.shape == (3,1)
    assert q.shape == (4,1)
    assert p_cov.shape == (9,9)    # q_curr = Quaternion(axis_angle=(gyro_data[k]-gyro_bias)*dt)


    # 2.1 Compute Kalman Gain
    r_cov = sensor_var * np.eye(3)
    H = np.zeros([3, 9])
    R = quat2rot(q)
    H[:, 3:6] = R.T# measurement model jacobian
    H[:, 6:9] = skew_symmetric(R.T@v)  # measurement model jacobian

    K = p_cov @ H.T @ np.linalg.inv(H@p_cov@H.T + r_cov)
    # 2.2 Compute error state
    delta_x = K @ (y - R.T@v)
    assert delta_x.shape == (9,1)

    # 2.3 Correct predicted state
    p_hat = p + delta_x[0:3]
    v_hat = v + delta_x[3:6]
    # q_hat = Quaternion(axis_angle=delta_x[6:9]).quat_mult_right(Quaternion(*q))
    q_hat = rot2quat(R@expm(skew_symmetric(delta_x[6:9]))).reshape((4,1))

    # 2.4 Compute corrected covariance
    p_cov_hat = (np.eye(9) - K@H) @ p_cov
    quat2rot
    assert p_hat.shape == (3,1)
    assert v_hat.shape == (3,1)
    assert q_hat.shape == (4,1)
    assert p_cov_hat.shape == (9,9)

    return p_hat, v_hat, q_hat, p_cov_hat




# t0 = time.time()
# # bag = rosbag.Bag('/media/jonathan/SamsungSSD1/tro_data/neya/neya_data/follow_general_2022-12-01-15-44-43_0.bag')
# # bag = rosbag.Bag('/media/justin/DATA/data/husky_data/2022-05-11_MAir/mair_trial1_rectangle.bag')
# # bag = rosbag.Bag('/media/justin/DATA/data/husky_data/2022-05-11_MAir/mair_trial9_M.bag')
# bag = rosbag.Bag('/media/justin/DATA/data/husky_data/northwood/2023-04-24-15-46-01.bag')

# t1 = time.time()
# # imu_data = read_imu(bag, '/mrzr/localization/imu/imu') # neya
# imu_data = read_imu(bag, '/gx5_1/imu/data') # husky
# t_imu = imu_data[:, 0]
# gyro_data = imu_data[:, 1:4]
# acc_data = imu_data[:, 4:7]
# # quat_data = imu_data[:, 7:11]
# # vel_data = read_vel(bag, "/mrzr/localization/wheel_encoder/twist") # neya
# vel_data = read_husky(bag, "/joint_states") # husky
# t_vel = vel_data[:, 0]
# vel_data = vel_data[:, 1:4]
# # R_imu2body = quat2rot([1., 0., 0., 0.]) # neya
# R_imu2body = quat2rot([0, 0.7071068, -0.7071068, 0]) # husky
# # gyro_data = ((gyro_data.T)).T
# # acc_data = ((acc_data.T)).T
# # quat_data = 

# t2 = time.time()
# print("Bag reading time: ", t1-t0)
# print("Process bag time: ", t2-t1)

# ##### neya #####
# # acc_bias = np.array([0.01,   -0.0642287,    -0.155932]).reshape(3)
# # gyro_bias = np.array([-7.49327e-05,  0.000315891,   0.00032303]).reshape(3)
# acc_cov = 0.01 # check in datasheet
# gyro_cov = 0.01 # check in datasheet
# wheel_encoder_cov = 0.034 # check in rosbag

# ##### husky mair #####
# # acc_bias = np.array([0.452427  , -0.0680003,  -0.00248014]) # husky mair
# # gyro_bias = np.array([0.0011257,  0.000182102, -0.000207972]) # husky mair

# acc_bias = np.array([ 0.361971,     0.249837,  -0.00240915]) # husky nw
# gyro_bias = np.array([0.0011879, -2.55878e-05,  -0.00049881  ]) # husky nw

# # acc_cov = 
# # gyro_cov =
# # wheel_encoder_cov = 

# g = np.array([0, 0, 9.81]).reshape(3)

# # X = [p, v, q]
# p_est = np.zeros([imu_data.shape[0], 3]) # position estimates
# v_est = np.zeros([imu_data.shape[0], 3]) # velocity estimates
# q_est = np.zeros([imu_data.shape[0], 4]) # quaternion estimates
# p_cov = np.zeros([imu_data.shape[0], 9, 9]) # state covariance

# p_est[0] = np.array([0, 0, 0])
# v_est[0] = np.array([0, 0, 0])
# q_est[0] = np.array([1, 0, 0, 0])
# # q_est[0] = np.array([ 0.9990482, 0.0436194, 0, 0 ])
# p_cov[0] = np.zeros([9, 9])

# # define measurement update for wheel encoder



# ##################################################################
# ################################################################## 

# i = 0
# for k in tqdm(range(1, t_imu.shape[0])): # start at 1 because we have initial state
#     # Time difference

#     dt = t_imu[k] - t_imu[k-1]

#     # 1. Update state with IMU inputs
#     # q_prev = Quaternion(*q_est[k-1, :])
#     # q_curr = Quaternion(axis_angle=(gyro_data[k]-gyro_bias)*dt)
#     # R_ns = q_prev.to_mat()
#     R_ns = quat2rot(q_est[k-1, :])
#     f_ns = (R_ns @ (R_imu2body @ acc_data[k]-acc_bias).reshape((3,1))) - g.reshape((3,1)) # acc_bias in body frame
#     # print(R_ns @ (R_imu2body @ acc_data[k]-acc_bias).reshape((3,1)))
#     # print(f_ns)
#     # print("-------")    
#     assert f_ns.shape == (3,1)

#     p_ = p_est[k-1, :].reshape((3,1)) + (v_est[k-1, :]*dt).reshape((3,1)) + 0.5*f_ns*dt**2 
#     v_ = v_est[k-1, :].reshape((3,1)) + f_ns*dt
#     # q_ = q_prev.quat_mult_left(q_curr).reshape((4,1))
#     R_predict = R_ns @ expm(skew_symmetric((R_imu2body@gyro_data[k]-gyro_bias)*dt))
#     # print(gyro_data[k])
#     # print("---")
#     q_ = rot2quat(R_predict).reshape((4,1))

#     # 1.1 get motion model Jacobian
#     # F = np.eye(9)
#     # F[0:3, 3:6] = np.eye(3)*dt
#     # F[3:6, 6:9] = -R_ns @ skew_symmetric((acc_data[k]-acc_bias))*dt
#     # F[6:9, 6:9] = Quaternion(aq_xis_angle=(gyro_data[k]-gyro_bias)*dt).to_mat().T

#     zero = np.zeros((3,3))
#     I = np.eye(3)
#     F = expm( np.block([[ zero ,   I,                                            zero],
#                         [ zero ,   zero, -R_predict @ skew_symmetric(R_imu2body@acc_data[k] - acc_bias)],
#                         [ zero ,   zero,      -skew_symmetric(R_imu2body@gyro_data[k]-gyro_bias)]])*dt )

#     # 1.2 uncertainty propagation
#     q_cov = np.zeros([9, 9]) # IMU noise covariance
#     q_cov[0:3, 0:3] = 0.000001*np.eye(3)
#     q_cov[3:6, 3:6] = acc_cov * np.eye(3) *dt**2
#     q_cov[6:9, 6:9] = gyro_cov * np.eye(3) *dt**2

#     l_jac = np.zeros((9,6))
#     l_jac[3:, :] = np.eye(6) # motion model noise jacobian
#     p_cov_ = F @ p_cov[k-1, :, :] @ F.T + q_cov 

#     # # 1.3 velocity update
#     # if(k<t_vel.shape[0] and abs(t_imu[k]-t_vel[i]) <= abs(t_imu[k+1]-t_vel[i]) and i<vel_data.shape[0]):
#     #     p_, v_, q_, p_cov_ = measurement_update(wheel_encoder_cov, vel_data[i, :].reshape((3,1)), p_cov_, p_, v_, q_)
#     #     i+=1
#     if i<vel_data.shape[0] and t_imu[k] >= t_vel[i]:
#         p_, v_, q_, p_cov_ = measurement_update(wheel_encoder_cov, vel_data[i, :].reshape((3,1)), p_cov_, p_, v_, q_)
#         i+=1


#     # 1.4 update states
#     p_est[k, :] = p_.T
#     v_est[k, :] = v_.T
#     q_est[k, :] = q_.T
#     p_cov[k, :, :] = p_cov_.T

# # print(i)   


# save_pose_to_tum(t_imu, p_est, q_est, filename='tum_estimated.txt')

