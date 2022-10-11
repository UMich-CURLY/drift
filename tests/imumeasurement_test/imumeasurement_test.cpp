#include <gtest/gtest.h>
#include "utils/imu.h"

TEST(ImuMeasurementTest, CtorTypeTest) {
  ImuMeasurement<double> imu_data;
  EXPECT_EQ(imu_data.get_type(), 1);
}

TEST(ImuMeasurementTest, CtorTypeTest2) {
  ImuMeasurement<double> imu_data;
  EXPECT_EQ(imu_data.get_type(), 0);
}
