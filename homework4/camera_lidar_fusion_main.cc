// Copyright @2018 Pony AI Inc. All rights reserved.

#include <iostream>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/opencv.hpp>

#include "common/utils/display/color.h"
#include "common/utils/file/path.h"
#include "common/utils/strings/format.h"
#include "homework4/camera_lidar_fusion_utils.h"

DEFINE_string(pony_data_dir, "", "The path of pony data.");
DEFINE_string(lidar_device, "VelodyneDevice32c", "");
DEFINE_string(camera_device, "GigECameraDeviceWideAngle", "");

void DrawPointCloudOnCameraImage(const PointCloud& pointcloud,
                                 const Eigen::VectorXd& intrinsic,
                                 const Eigen::Affine3d& extrinsic,
                                 cv::Mat* image) {
  const auto pixel_info = ProjectPointCloudToImage(pointcloud, intrinsic, extrinsic, 1920, 1080);
  for (const auto& pixel : pixel_info) {
    const cv::Point2d point(pixel.uv.u, pixel.uv.v);
    const double depth = pixel.position_in_camera_coordinate.norm();
    const double hue = (math::Clamp(depth, 1.0, 150.0) - 1.0) / (150.0 - 1.0);
    const Eigen::Vector3d rgb = utils::display::HsvToRgb(Eigen::Vector3d(hue, 1.0, 1.0)) * 255;
    cv::circle(*image, point, 0, cv::Scalar(rgb.z(), rgb.y(), rgb.x()), 2);
  }
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  const std::string intrinsic_file_name =
      strings::Format("car_param/{}_intrinsic", FLAGS_camera_device);
  const std::string intrinsic_file = file::path::Join(FLAGS_pony_data_dir, intrinsic_file_name);
  CHECK(file::path::Exists(intrinsic_file)) << intrinsic_file << " doesn't exist!";
  const std::string extrinsic_file_name =
      strings::Format("car_param/lidar_to_{}_extrinsic", FLAGS_camera_device);
  const std::string extrinsic_file = file::path::Join(FLAGS_pony_data_dir, extrinsic_file_name);
  CHECK(file::path::Exists(extrinsic_file)) << extrinsic_file << " doesn't exist!";

  const Eigen::VectorXd intrinsic = ReadCameraIntrinsic(intrinsic_file);
  const Eigen::Affine3d extrinsic = ReadRigidBodyTransform(extrinsic_file);
  int frame_index = 0;
  cv::namedWindow("fusion demo", cv::WINDOW_NORMAL);
  while (true) {
    const std::string image_path = file::path::Join(
        FLAGS_pony_data_dir, strings::Format("select/{}/{}.jpg", FLAGS_camera_device, frame_index));
    if (!file::path::Exists(image_path)) {
      break;
    }
    cv::Mat image = cv::imread(image_path);
    const std::string pc_path = file::path::Join(
        FLAGS_pony_data_dir, strings::Format("select/{}/{}.txt", FLAGS_lidar_device, frame_index));
    if (!file::path::Exists(pc_path)) {
      break;
    }
    const auto pc = ReadPointCloudFromTextFile(pc_path);
    DrawPointCloudOnCameraImage(pc, intrinsic, extrinsic, &image);
    cv::putText(image, image_path, cv::Point(10, 30),
                cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0));
    cv::imshow("fusion demo", image);
    cv::waitKey(0);
    ++frame_index;
  }
  return 0;
}
