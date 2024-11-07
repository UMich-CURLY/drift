/* ----------------------------------------------------------------------------
 * Copyright 2023, CURLY Lab, University of Michigan
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   inekf_estimator.h
 *  @author Tzu-Yuan Lin, Tingjun Li
 *  @brief  Header file for state estimator class
 *  @date   May 16, 2023
 **/

// #ifdef UTILS_TYPE_DEF_H
// #define UTILS_TYPE_DEF_H

#include <queue>

#include "drift/measurement/angular_velocity.h"
#include "drift/measurement/imu.h"
#include "drift/measurement/legged_kinematics.h"
#include "drift/measurement/navsat.h"
#include "drift/measurement/odom.h"
#include "drift/measurement/velocity.h"
#include "drift/state/robot_state.h"

using namespace measurement;
using namespace state;

// Robot State:
typedef std::queue<std::shared_ptr<RobotState>>
    RobotStateQueue; /**< Queue of pointers to robot state */
typedef std::shared_ptr<RobotStateQueue>
    RobotStateQueuePtr; /**< Pointer to the robot state queue */

// Limits:
typedef std::numeric_limits<double> dbl;

/* ====================================== */
// Measurements:
/* ====================================== */
typedef std::queue<std::shared_ptr<Measurement>>
    MeasurementQueue; /**< Queue for storing sensor data. */
typedef std::shared_ptr<MeasurementQueue>
    MeasurementQueuePtr; /**< Pointer to the queue for storing sensor data. */

// IMU:
typedef std::shared_ptr<ImuMeasurement<double>>
    ImuMeasurementPtr; /**< Shared pointer to a ImuMeasurement object. */
typedef std::queue<ImuMeasurementPtr>
    IMUQueue; /**< Queue of ImuMeasurementPtr objects. */
typedef std::shared_ptr<IMUQueue> IMUQueuePtr; /**< Shared pointer to a
                                                  IMUQueue object. */

// Legged Kinematics:
typedef std::shared_ptr<LeggedKinematicsMeasurement>
    LeggedKinMeasurementPtr; /**< Type: Shared pointer to a
                                 LeggedKinematicsMeasurement object. */
typedef std::queue<LeggedKinMeasurementPtr>
    LeggedKinQueue; /**< Type: Queue of LeggedKinMeasurementPtr
                     * objects.
                     */
typedef std::shared_ptr<LeggedKinQueue>
    LeggedKinQueuePtr; /**< Type: Shared pointer
                                 to a KinematicsQueue object. */

// Velocity:
typedef std::shared_ptr<VelocityMeasurement<double>>
    VelocityMeasurementPtr; /**< Type: Shared pointer to a VelocityMeasurement
                               object. */
typedef std::queue<VelocityMeasurementPtr>
    VelocityQueue; /**< Type: Queue of VelocityMeasurementPtr objects. */
typedef std::shared_ptr<VelocityQueue>
    VelocityQueuePtr; /**< Type: Shared pointer to a VelocityQueue object. */

// Angular Velocity:
typedef std::shared_ptr<AngularVelocityMeasurement<double>>
    AngularVelocityMeasurementPtr; /**< Pointer to the
                                      AngularVelocityMeasurement object. */
typedef std::queue<AngularVelocityMeasurementPtr>
    AngularVelocityQueue; /**< Queue for storing angular velocity measurements.
                           */
typedef std::shared_ptr<AngularVelocityQueue> AngularVelocityQueuePtr; /**<
Pointer to the AngularVelocityQueue. */

// Odom:
typedef std::shared_ptr<OdomMeasurement> OdomMeasurementPtr; /**< Pointer to the
                                                                OdomMeasurement
                                                                object. */
typedef std::queue<OdomMeasurementPtr> OdomQueue; /**< Queue for storing
                                                     odometry measurements. */
typedef std::shared_ptr<OdomQueue> OdomQueuePtr;  /**< Pointer to the
                                                       OdomQueue. */


// #endif    // UTILS_TYPE_DEF_H
