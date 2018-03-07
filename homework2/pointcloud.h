// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <fstream>
#include <string>
#include <vector>

#include <Eigen/Core>
#include <glog/logging.h>

struct PointCloud {
  // Lidar points in its local coordinate system.
  std::vector<Eigen::Vector3d> points;
  // Rotation and Translation to transform Lidar points to world coordinate system.
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

PointCloud ReadPointCloudFromTxtFile(const std::string& file_name);
