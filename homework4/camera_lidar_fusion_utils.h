// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <glog/logging.h>

#include "common/utils/math/math_utils.h"
#include "homework2/pointcloud.h"

// u corresponds to x on image; v corresponds to y on image;
// https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
struct ImageUV {
  double u = 0.0;
  double v = 0.0;

  ImageUV(double u, double v) : u(u), v(v) {}
};

struct PixelInfo {
  // Pixel coordinate on image.
  ImageUV uv;
  // Point 3D position in camera coordinate system.
  Eigen::Vector3d position_in_camera_coordinate;

  PixelInfo(const ImageUV& uv, const Eigen::Vector3d& position_in_camera_coordinate)
      : uv(uv), position_in_camera_coordinate(position_in_camera_coordinate) {}
};

Eigen::VectorXd ReadCameraIntrinsic(const std::string& file_name);

// The transform is from Lidar local coordinate system to Camera's
Eigen::Affine3d ReadRigidBodyTransform(const std::string& file_name);

// Project 3D point in camera coordinate system to image plane.
ImageUV Project3dPointToImage(const Eigen::Vector3d& point,
                              const Eigen::VectorXd& camera_intrinsic);

// Project Pointcloud from Lidar local coordinate system to image plane.
std::vector<PixelInfo> ProjectPointCloudToImage(const PointCloud& pointcloud,
                                                const Eigen::VectorXd& intrinsic,
                                                const Eigen::Affine3d& extrinsic,
                                                int image_width, int image_height);
