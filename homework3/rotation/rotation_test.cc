// Copyright @2018 Pony AI Inc. All rights reserved.
#include <gtest/gtest.h>

#include "homework3/rotation/rotation.h"

namespace homework3 {
namespace {

// Both functions should not crash even for no rotation.
TEST(Rotation, BasicTest) {
  Eigen::Matrix3d rotation;
  rotation << 1, 0, 0,
              0, 1, 0,
              0, 0, 1;
  auto roll_pitch_yaw = ToRollPitchYaw(rotation);
  auto angle_axis = ToAngleAxis(rotation);
  EXPECT_DOUBLE_EQ(0.0, roll_pitch_yaw.x());
  EXPECT_DOUBLE_EQ(0.0, angle_axis.angle());
}

TEST(Rotation, RollPitchYawTest) {
  Eigen::Matrix3d rotation;
  rotation << 0.97517033, -0.0978434, -0.19866933,
              0.03695701, 0.95642509, -0.28962948,
              0.21835066, 0.27509585, 0.93629336;
  auto roll_pitch_yaw = ToRollPitchYaw(rotation);
  EXPECT_NEAR(0.1, roll_pitch_yaw.x(), 1e-3);
  EXPECT_NEAR(-0.2, roll_pitch_yaw.y(), 1e-3);
  EXPECT_NEAR(0.3, roll_pitch_yaw.z(), 1e-3);
}

TEST(Rotation, AngleAxisTest) {
  Eigen::Matrix3d rotation;
  rotation << 0.99505412, -0.03004994, 0.01981681,
              0.02985011, 0.995204, 0.01028309,
              -0.02011656, -0.00968359, 0.99545379;
  auto angle_axis = ToAngleAxis(rotation);
  EXPECT_NEAR(1.0 angle_axis.axis().length(), 3e-2)
  EXPECT_NEAR(0.1, std::abs(angle_axis.angle()), 3e-2);
  EXPECT_NEAR(-2 * angle_axis.angle() * angle_axis.axis().x(),
              angle_axis.angle() * angle_axis.axis().y(), 3e-2);
  EXPECT_NEAR(-3 * angle_axis.angle() * angle_axis.axis().x(),
              angle_axis.angle() * angle_axis.axis().z(), 3e-2);
}


}  // namespace
}  // namespace homework3

