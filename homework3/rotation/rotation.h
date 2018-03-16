// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "Eigen/Core"
#include "Eigen/Geometry"

namespace homework3 {

Eigen::Vector3d ToRollPitchYaw(Eigen::Matrix3d rotation);

Eigen::AngleAxisd ToAngleAxis(Eigen::Matrix3d rotation);

}  // namespace homework3
