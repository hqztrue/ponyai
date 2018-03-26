// Copyright @2018 Pony AI Inc. All rights reserved.


#include "camera_lidar_fusion_utils.h"

Eigen::VectorXd ReadCameraIntrinsic(const std::string& file_name) {
  Eigen::VectorXd intrinsic = Eigen::VectorXd::Zero(9);
  FILE* fin = CHECK_NOTNULL(std::fopen(file_name.c_str(), "r"));
  int num_read = std::fscanf(fin, "%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf,%lf\n",
                             &intrinsic(0), &intrinsic(1), &intrinsic(2),
                             &intrinsic(3), &intrinsic(4), &intrinsic(5),
                             &intrinsic(6), &intrinsic(7), &intrinsic(8));
  CHECK_EQ(9, num_read);
  std::fclose(fin);
  return intrinsic;
}

Eigen::Affine3d ReadRigidBodyTransform(const std::string& file_name) {
  FILE* fin = CHECK_NOTNULL(std::fopen(file_name.c_str(), "r"));
  Eigen::Vector3d translation = Eigen::Vector3d::Zero();
  int num_read = std::fscanf(fin, "%lf,%lf,%lf\n",
                             &translation(0), &translation(1), &translation(2));
  CHECK_EQ(3, num_read);

  Eigen::Matrix3d rotation = Eigen::Matrix3d::Zero();
  num_read = std::fscanf(fin, "%lf,%lf,%lf\n%lf,%lf,%lf\n%lf,%lf,%lf\n",
                         &rotation(0, 0), &rotation(0, 1), &rotation(0, 2),
                         &rotation(1, 0), &rotation(1, 1), &rotation(1, 2),
                         &rotation(2, 0), &rotation(2, 1), &rotation(2, 2));
  CHECK_EQ(9, num_read);

  Eigen::Affine3d affine3d = Eigen::Affine3d::Identity();
  affine3d.translation() = translation;
  affine3d.linear() = rotation;
  return affine3d;
}

ImageUV Project3dPointToImage(const Eigen::Vector3d& point,
                              const Eigen::VectorXd& camera_intrinsic) {
  const double k1 = camera_intrinsic(0);
  const double k2 = camera_intrinsic(1);
  const double k3 = camera_intrinsic(2);
  const double p1 = camera_intrinsic(3);
  const double p2 = camera_intrinsic(4);
  const double fx = camera_intrinsic(5);
  const double fy = camera_intrinsic(6);
  const double cx = camera_intrinsic(7);
  const double cy = camera_intrinsic(8);

  double u = 0.0;
  double v = 0.0;
  /*
   * Implement the code to project 3D point in camera coordinate system to image plane.
   * https://docs.opencv.org/2.4/modules/calib3d/doc/camera_calibration_and_3d_reconstruction.html
   */
  return {u, v};
}

std::vector<PixelInfo> ProjectPointCloudToImage(
    const PointCloud& pointcloud,
    const Eigen::VectorXd& intrinsic,
    const Eigen::Affine3d& extrinsic,
    int image_width, int image_height) {
  std::vector<PixelInfo> pixel_info;
  /*
   * Implement the code to project Pointcloud from Lidar local coordinate system to image plane.
   * NOTE, consider to filter out the points which are not within the camera FOV (field of view)
   * before projecting them onto image plane. Pointcloud's horizontal FOV is 360 degree and our
   * camera's horizontal FOV is about 80 here.
   */
  return pixel_info;
}
