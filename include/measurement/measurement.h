/* ----------------------------------------------------------------------------
 * Copyright 2018, Ross Hartley <m.ross.hartley@gmail.com>
 * All Rights Reserved
 * See LICENSE for the license information
 * -------------------------------------------------------------------------- */

/**
 *  @file   measurement.h
 *  @author Ross Hartley
 *  @brief  Header file for Measurement class
 *  @date   September 27, 2018
 **/

#ifndef MEASUREMENT_H
#define MEASUREMENT_H

#include <Eigen/Dense>
#include <string>
#include "core/InEKF.h"
// #include "tf/transform_listener.h"

enum MeasurementType {EMPTY, IMU, KINEMATICS, CONTACT, JOINT_STATE};

class Measurement {
    struct MeasurementHeader {
        uint64_t seq;
        double stamp;
        std::string frame_id;
    };

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW
        Measurement();
        virtual ~Measurement() = default;

        MeasurementHeader header;

        double getTime();
        MeasurementType getType();

        friend std::ostream& operator<<(std::ostream& os, const Measurement& m);  

    protected:
        MeasurementType type_;
};

struct MeasurementCompare {
  bool operator()(Measurement& lhs, Measurement& rhs) const {
    return lhs.getTime() > rhs.getTime();
  }
};

#endif 
