// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception.h"

interface::perception::PerceptionObstacles Perception::RunPerception(
    const PointCloud& pointcloud, const utils::Optional<cv::Mat>& image) {
  interface::perception::PerceptionObstacles perception_result;
  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(976.05);
      polygon_point->set_y(1079.60);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(974.76);
      polygon_point->set_y(1087.13);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(972.22);
      polygon_point->set_y(1086.69);
      polygon_point->set_z(-8.13);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(973.52);
      polygon_point->set_y(1079.16);
      polygon_point->set_z(-8.13);
    }
    obstacle->set_height(2.69);
    obstacle->set_id("c83");
  }

  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::CAR);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(972.10);
      polygon_point->set_y(1084.33);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(971.20);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.00);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(969.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c84");
  }

  // Add a mocked up obstacle.
  {
    auto* obstacle = perception_result.add_obstacle();
    obstacle->set_type(interface::perception::ObjectType::UNKNOWN_TYPE);
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(967.10);
      polygon_point->set_y(1084.3295967224144);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(965.19);
      polygon_point->set_y(1088.79);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.01);
      polygon_point->set_y(1088.34);
      polygon_point->set_z(-8.18);
    }
    {
      auto* polygon_point = obstacle->add_polygon_point();
      polygon_point->set_x(964.91);
      polygon_point->set_y(1083.89);
      polygon_point->set_z(-8.18);
    }
    obstacle->set_height(1.56);
    obstacle->set_id("c88");
  }

  if (image) {
    // Remove me if you don't want to pause the program every time.
    cv::namedWindow("camera");
    imshow("camera", *image);
    cv::waitKey(0);
  }

  LOG(INFO) << "Perception done.";
  return perception_result;
}