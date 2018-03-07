// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <string>
#include <vector>

#include <Eigen/Core>

struct PointCloud {
  // Lidar points in its local coordinate system.
  std::vector<Eigen::Vector3d> points;

  // Rotation and translation to transform LiDAR points to world coordinate system.
  Eigen::Matrix3d rotation;
  Eigen::Vector3d translation;
};

PointCloud ReadPointCloudFromTextFile(const std::string& file_name);
