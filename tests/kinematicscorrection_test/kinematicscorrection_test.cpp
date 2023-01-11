
// STL
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/socket.h>
#include <time.h>
#include <unistd.h>
#include <chrono>
#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include "filter/base_correction.h"
#include "filter/base_propagation.h"
#include "filter/inekf/correction/kinematics_correction.h"
#include "filter/inekf/propagation/imu_propagation.h"
#include "measurement/imu.h"
#include "measurement/kinematics.h"
#include "state/robot_state.h"
#include "state_estimator.h"

// Boost
#include <boost/algorithm/string.hpp>
// Threading
#include <boost/circular_buffer.hpp>
#include <boost/thread/condition.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/thread.hpp>

// Testing
#include <gtest/gtest.h>


TEST(KinematicsCorrection, ImuPropVelCorr) {}