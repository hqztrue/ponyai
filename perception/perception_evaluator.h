// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <ctime>
#include <iomanip>
#include <iostream>
#include <memory>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/proto/object_labeling_3d.pb.h"
#include "common/proto/perception_evaluation.pb.h"
#include "common/utils/evaluation/grading.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/hungarian/hungarian_sparse.h"
#include "common/utils/math/transform/transform.h"
#include "common/utils/math/vec2d.h"
#include "common/utils/strings/format.h"
#include "homework2/pointcloud.h"
#include "perception/perception.h"

class PerceptionEvaluator {
 public:
  struct Options {
    std::string scenario_name;
    std::string lidar_folder;
    std::string camera_folder;
    std::unordered_map<std::string, std::string> data_to_label_map;
    double evaluation_range;
  };

  struct PerObstacleMetrics {
    double jaccard = 0.0;
    double precision = 0.0;
    double recall = 0.0;
  };

  struct OverallPrecisionRecall {
    double precision = 0.0;
    double recall = 0.0;
  };

  explicit PerceptionEvaluator(Options options);

  virtual ~PerceptionEvaluator() = default;

  interface::perception::PerceptionEvaluationResult RunEvaluation();

 private:
  using Polygon2d = std::vector<math::Vec2d>;

  struct ObstacleInfo {
    std::string id;
    Polygon2d polygon;
    interface::perception::ObjectType type = interface::perception::ObjectType::UNKNOWN_TYPE;
    std::vector<int> lidar_points;
    double height = 0.0;
    double floor_z = 0.0;

    interface::perception::EffectivePolygonInfo ToEffectivePolygonInfo() const {
      interface::perception::EffectivePolygonInfo polygon_info;
      polygon_info.set_id(id);
      polygon_info.set_type(interface::perception::ObjectType_Name(type));
      polygon_info.set_floor(floor_z);
      polygon_info.set_ceiling(height + floor_z);
      for (const math::Vec2d& point : polygon) {
        auto* pt = polygon_info.add_point();
        pt->set_x(point.x);
        pt->set_y(point.y);
      }
      return polygon_info;
    }
  };

  using SimilarityMatrix = std::vector<std::vector<PerObstacleMetrics>>;

  interface::perception::PerceptionFrameResult RunEvaluationOnFrame(
      const interface::object_labeling::ObjectLabels& object_labels,
      const interface::perception::PerceptionObstacles& perception_obstacles,
      const PointCloud& pointcloud);

  std::vector<ObstacleInfo> ConstructObstacleInfoForLabeledObstacles(
      const interface::object_labeling::ObjectLabels& labeled_obstacles,
      const PointCloud& pointcloud);

  std::vector<ObstacleInfo> ConstructObstacleInfoForDetectedObstacles(
      const interface::perception::PerceptionObstacles& perception_obstacles,
      const PointCloud& pointcloud);

  std::vector<ObstacleInfo> FilterObstacleInfoBasedOnEvaluationCondition(
      const std::vector<ObstacleInfo>& obstacles,
      const math::transform::Rigid3d& lidar_pose,
      double range) const;

  SimilarityMatrix ComputeSimilarityMatrix(const std::vector<ObstacleInfo>& labeled,
                                           const std::vector<ObstacleInfo>& detected);

  OverallPrecisionRecall CalculateOverallPrecisionRecall(const SimilarityMatrix& similarity);

  ObstacleInfo ConstructObstacleInfo(const std::string& id,
                                     double height,
                                     double floor_z,
                                     const Polygon2d& polygon,
                                     const interface::perception::ObjectType& type,
                                     const PointCloud& pointcloud) const;

  bool IsObstacleMeetsEvaluationCondition(const ObstacleInfo& obstacle_info,
                                          const Eigen::Vector3d& lidar_pos,
                                          double range) const;

  PerObstacleMetrics ComputePerObstacleMetrics(const ObstacleInfo& labeled,
                                               const ObstacleInfo& detected);

  std::vector<int> ComputeLidarPoints(const PointCloud& pointcloud, const Polygon2d& polygon) const;

  Options options_;
  Perception perception_;

  DISALLOW_COPY_MOVE_AND_ASSIGN(PerceptionEvaluator);
};
