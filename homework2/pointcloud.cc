// Copyright @2018 Pony AI Inc. All rights reserved.

#include "homework2/pointcloud.h"

#include <fstream>

#include <glog/logging.h>

PointCloud ReadPointCloudFromTextFile(const std::string& file_name) {
  FILE* file = std::fopen(file_name.c_str(), "r");
  CHECK(file != nullptr) << "Fail to open file " << file_name;

  PointCloud pointcloud;
  Eigen::Vector3d& translation = pointcloud.translation;
  int num_read = std::fscanf(file, "%lf,%lf,%lf\n",
                             &translation(0), &translation(1), &translation(2));
  CHECK_EQ(3, num_read) << "Fail to read translation from file " << file_name;

  Eigen::Matrix3d& rotation = pointcloud.rotation;
  num_read = std::fscanf(file, "%lf,%lf,%lf\n%lf,%lf,%lf\n%lf,%lf,%lf\n",
                         &rotation(0, 0), &rotation(0, 1), &rotation(0, 2),
                         &rotation(1, 0), &rotation(1, 1), &rotation(1, 2),
                         &rotation(2, 0), &rotation(2, 1), &rotation(2, 2));
  CHECK_EQ(9, num_read) << "Fail to read rotation from file " << file_name;

  Eigen::Vector3d point = Eigen::Vector3d::Zero();
  int point_index = 0;
  while (std::fscanf(file, "%d,%lf,%lf,%lf\n",
                     &point_index, &point(0), &point(1), &point(2)) != EOF) {
    pointcloud.points.emplace_back(point);
  }
  std::fclose(file);

  return pointcloud;
}
