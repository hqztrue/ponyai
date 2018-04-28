// Copyright @2018 Pony AI Inc. All rights reserved.

#include "perception/perception_evaluator.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/utils/common/optional.h"

DEFINE_string(perception_evaluation_label_file_suffix, "label", "Label file suffix");
DEFINE_string(perception_evaluation_data_file_suffix, "txt", "pointcloud data file suffix");

using file::path::FilenameStem;
using hungarian::HungarianSparse;
using interface::object_labeling::ObjectLabel;
using interface::object_labeling::ObjectLabels;
using interface::perception::EffectivePolygonInfo;
using interface::perception::ObjectType;
using interface::perception::PerceptionObstacles;
using interface::perception::PerceptionEvaluationResult;
using interface::perception::PerceptionFrameResult;
using math::transform::Rigid3d;
using math::Vec2d;

namespace {

math::transform::Rigid3d GetLidarPoseFromPointCloud(const PointCloud& pointcloud) {
  return Rigid3d(pointcloud.translation, Eigen::Quaterniond(pointcloud.rotation));
}

ObjectType ConvertToInternalObjectType(const ObjectType& object_type) {
  std::string str = ObjectType_Name(object_type);
  if (strings::StartWith(str, "UNKNOWN")) {
    return ObjectType::UNKNOWN_TYPE;
  } else if (strings::StartWith(str, "CAR")) {
    return ObjectType::CAR;
  } else if (strings::StartWith(str, "PEDESTRIAN")) {
    return ObjectType::PEDESTRIAN;
  } else if (strings::StartWith(str, "CYCLIST")) {
    return ObjectType::CYCLIST;
  } else {
    LOG(FATAL) << "invalid object_type: " << object_type;
    return ObjectType::UNKNOWN_TYPE;
  }
}

template<class Container>
std::vector<Vec2d> ToPolygon2d(const Container& points) {
  std::vector<Vec2d> polygon;
  for (const auto& point : points) {
    polygon.emplace_back(math::ToVec2d(point));
  }
  return polygon;
}

bool IsPointInOrOnBoundaryPolygon(const std::vector<Vec2d>& polygon, const Vec2d& point) {
  int num_points = static_cast<int>(polygon.size());
  if (num_points < 3) {
    return false;
  }
  double x0 = point.x;
  double y0 = point.y;
  bool result = false;
  int j = num_points - 1;
  for (int i = 0; i < num_points; ++i) {
    const Vec2d& p1 = polygon[i];
    const Vec2d& p2 = polygon[j];
    if (p1.DistanceToPoint(point) <= math::kEpsilon) {
      return true;
    }
    double d = p1.DistanceToPoint(p2);
    if (d >= math::kEpsilon && InnerProd(p1, p2, point) >= 0.0 && InnerProd(p2, p1, point) >= 0.0 &&
        std::abs(OuterProd(p1, p2, point)) <= d * math::kEpsilon) {
      return true;
    }
    if ((p1.y > y0) != (p2.y > y0)) {
      const double dy = p1.y - p2.y;
      const double dx = p1.x - p2.x;
      const double outer = (x0 - p1.x) * dy - (y0 - p1.y) * dx;
      if ((dy > 0) ? (outer < 0) : (outer > 0)) {
        result = !result;
      }
    }
    j = i;
  }
  return result;
}

}  // namespace

PerceptionEvaluator::PerceptionEvaluator(Options options)
    : options_(std::move(options)) {
}

PerceptionEvaluationResult PerceptionEvaluator::RunEvaluation() {
  PerceptionEvaluationResult evaluation_result;
  evaluation_result.set_log_directory(options_.scenario_name);
  evaluation_result.set_evaluation_range(options_.evaluation_range);
  std::vector<std::string> pointcloud_files = file::path::FindFilesWithPrefixSuffix(
      options_.lidar_folder, "", FLAGS_perception_evaluation_data_file_suffix);
  std::sort(pointcloud_files.begin(), pointcloud_files.end(), file::path::Compare);

  for (const auto& pointcloud_file : pointcloud_files) {
    const PointCloud pointcloud = ReadPointCloudFromTextFile(pointcloud_file);
    // Load camera image if there is a corresponding one.
    const std::string image_file =
        file::path::Join(options_.camera_folder,
                         strings::Format("{}.jpg", file::path::FilenameStem(pointcloud_file)));
    utils::Optional<cv::Mat> image;
    if (file::path::Exists(image_file)) {
      image.emplace(cv::imread(image_file, CV_LOAD_IMAGE_COLOR));
    }
    // Run perception and save the results.
    const auto perception_obstacles = perception_.RunPerception(pointcloud, image);
    // Run evaluation when label file exists.
    if (options_.data_to_label_map.count(pointcloud_file)) {
      LOG(INFO) << "Evaluate data file: " << pointcloud_file;
      const auto& label_file = options_.data_to_label_map.at(pointcloud_file);
      // Load object labels.
      ObjectLabels object_labels;
      CHECK(file::ReadFileToProto(label_file, &object_labels));
      auto frame_result = RunEvaluationOnFrame(object_labels, perception_obstacles, pointcloud);
      // Save the data and label path for visualization.
      frame_result.set_label_file_path(label_file);
      frame_result.set_log_file_path(pointcloud_file);
      frame_result.set_log_file_id(object_labels.log_file_id());
      if (image) {
        frame_result.set_image_file_path(image_file);
      }
      *(evaluation_result.add_frame_result()) = frame_result;
    }
  }
  return evaluation_result;
}

PerceptionFrameResult PerceptionEvaluator::RunEvaluationOnFrame(
    const ObjectLabels& object_labels,
    const PerceptionObstacles& perception_obstacles,
    const PointCloud& pointcloud) {
  const Rigid3d lidar_pose = GetLidarPoseFromPointCloud(pointcloud);
  const auto labeled_obstacles =
      ConstructObstacleInfoForLabeledObstacles(object_labels, pointcloud);
  const auto detected_obstacles =
      ConstructObstacleInfoForDetectedObstacles(perception_obstacles, pointcloud);
  // Filter labeled and detected obstacles which are not under our evaluation conditions.
  auto filtered_labeled_obstacles = FilterObstacleInfoBasedOnEvaluationCondition(
      labeled_obstacles, lidar_pose, options_.evaluation_range);
  auto filtered_detected_obstacles = FilterObstacleInfoBasedOnEvaluationCondition(
      detected_obstacles, lidar_pose, options_.evaluation_range);
  // Compute similarity matrix for labeled and detected obstacles.
  const auto similarity_matrix = ComputeSimilarityMatrix(
      filtered_labeled_obstacles, filtered_detected_obstacles);
  // Calculate overall precision and recall.
  OverallPrecisionRecall precision_recall;
  if (detected_obstacles.size() != 0 || labeled_obstacles.size() != 0) {
    precision_recall = CalculateOverallPrecisionRecall(similarity_matrix);
  }
  PerceptionFrameResult frame_result;
  frame_result.set_precision(precision_recall.precision);
  frame_result.set_recall(precision_recall.recall);
  // Save the effective polygons for both labeled and detected obstacles for visualization purpose.
  for (const auto& labeled_obstacle : filtered_labeled_obstacles) {
    *frame_result.add_labeled_polygon() = labeled_obstacle.ToEffectivePolygonInfo();
  }
  for (const auto& detected_obstacle : filtered_detected_obstacles) {
    *frame_result.add_detected_polygon() = detected_obstacle.ToEffectivePolygonInfo();
  }
  return frame_result;
}

std::vector<PerceptionEvaluator::ObstacleInfo>
PerceptionEvaluator::ConstructObstacleInfoForLabeledObstacles(
    const ObjectLabels& labeled_obstacles,
    const PointCloud& pointcloud) {
  std::vector<ObstacleInfo> labeled_obstacle_info;
  labeled_obstacle_info.reserve(labeled_obstacles.object_size());
  for (const auto& label : labeled_obstacles.object()) {
    const ObjectType object_type = ConvertToInternalObjectType(label.type());
    CHECK(!label.polygon().point().empty());
    labeled_obstacle_info.emplace_back(
        ConstructObstacleInfo(label.id(),
                              label.height(),
                              label.polygon().point(0).z(),
                              ToPolygon2d(label.polygon().point()),
                              object_type,
                              pointcloud));
  }
  return labeled_obstacle_info;
}

std::vector<PerceptionEvaluator::ObstacleInfo>
PerceptionEvaluator::ConstructObstacleInfoForDetectedObstacles(
    const PerceptionObstacles& perception_obstacles,
    const PointCloud& pointcloud) {
  std::vector<ObstacleInfo> detected_obstacle_info;
  detected_obstacle_info.reserve(perception_obstacles.obstacle_size());
  for (const auto& obstacle_detected : perception_obstacles.obstacle()) {
    const auto& obstacle_type = obstacle_detected.type();
    CHECK(!obstacle_detected.polygon_point().empty());
    detected_obstacle_info.emplace_back(
        ConstructObstacleInfo(obstacle_detected.id(),
                              obstacle_detected.height(),
                              obstacle_detected.polygon_point(0).z(),
                              ToPolygon2d(obstacle_detected.polygon_point()),
                              obstacle_type,
                              pointcloud));
  }
  return detected_obstacle_info;
}

std::vector<PerceptionEvaluator::ObstacleInfo>
PerceptionEvaluator::FilterObstacleInfoBasedOnEvaluationCondition(
    const std::vector<ObstacleInfo>& obstacles,
    const math::transform::Rigid3d& lidar_pose,
    double range) const {
  std::vector<ObstacleInfo> filtered_obstacles;
  for (const auto& obstacle : obstacles) {
    if (!IsObstacleMeetsEvaluationCondition(obstacle, lidar_pose.translation(), range)) {
      continue;
    }
    if (obstacle.lidar_points.empty()) {
      continue;
    }
    if (obstacle.type == ObjectType::UNKNOWN_TYPE) {
      continue;
    }
    filtered_obstacles.push_back(obstacle);
  }
  return filtered_obstacles;
}

PerceptionEvaluator::SimilarityMatrix PerceptionEvaluator::ComputeSimilarityMatrix(
    const std::vector<ObstacleInfo>& labeled,
    const std::vector<ObstacleInfo>& detected) {
  // SimilarityMatrix is guaranteed to a matrix, which has size #labeled * #detected.
  SimilarityMatrix similarity;
  similarity.reserve(labeled.size());
  for (int i = 0; i < labeled.size(); ++i) {
    similarity.emplace_back();
    auto& row = similarity.back();
    row.reserve(detected.size());
    for (int j = 0; j < detected.size(); ++j) {
      row.push_back(ComputePerObstacleMetrics(labeled[i], detected[j]));
    }
  }
  return similarity;
}

PerceptionEvaluator::OverallPrecisionRecall
PerceptionEvaluator::CalculateOverallPrecisionRecall(const SimilarityMatrix& similarity) {
  OverallPrecisionRecall precision_recall;
  // SimilarityMatrix has size #labeled * #detected.
  const int num_labeled = similarity.size();
  if (num_labeled == 0) {
    return precision_recall;
  }
  const int num_detected = similarity[0].size();
  std::vector<HungarianSparse::Edge> edges;
  for (int i = 0; i < num_labeled; ++i) {
    for (int j = 0; j < num_detected; ++j) {
      if (similarity[i][j].jaccard <= math::kEpsilon) {
        continue;
      }
      HungarianSparse::Edge edge{i, j, -static_cast<int64_t>(similarity[i][j].jaccard * 1e6)};
      edges.push_back(edge);
    }
  }
  double best_matched_score = -static_cast<double>(HungarianSparse::MinCostMatching(
      num_labeled, num_detected, edges)) / 1e6;
  precision_recall.precision = num_detected > 0 ? best_matched_score / num_detected : 0.0;
  precision_recall.recall = num_labeled > 0 ? best_matched_score / num_labeled : 0.0;
  return precision_recall;
}

PerceptionEvaluator::ObstacleInfo PerceptionEvaluator::ConstructObstacleInfo(
    const std::string& id, double height, double floor_z,
    const Polygon2d& polygon, const ObjectType& type, const PointCloud& pointcloud) const {
  CHECK(!polygon.empty()) << "Invalid obstacle info found: " << id;
  ObstacleInfo obstacle_info;
  obstacle_info.id = id;
  obstacle_info.type = type;
  obstacle_info.polygon = polygon;
  obstacle_info.height = height;
  obstacle_info.floor_z = floor_z;
  obstacle_info.lidar_points = ComputeLidarPoints(pointcloud, polygon);
  return obstacle_info;
}

bool PerceptionEvaluator::IsObstacleMeetsEvaluationCondition(
    const ObstacleInfo& obstacle_info, const Eigen::Vector3d& lidar_pos, double range) const {
  const math::Vec2d pos_xy(lidar_pos.x(), lidar_pos.y());
  for (const math::Vec2d& point : obstacle_info.polygon) {
    if (point.DistanceToPoint(pos_xy) > range) {
      return false;
    }
  }
  return true;
}

PerceptionEvaluator::PerObstacleMetrics
PerceptionEvaluator::ComputePerObstacleMetrics(const ObstacleInfo& labeled,
                                               const ObstacleInfo& detected) {
  double overlap = 0.0;
  double obstacle1 = 0.0;
  double obstacle2 = 0.0;
  obstacle1 = labeled.lidar_points.size();
  obstacle2 = detected.lidar_points.size();
  overlap = grading::GetOverlapSizeOfSortedLists(labeled.lidar_points, detected.lidar_points);
  PerceptionEvaluator::PerObstacleMetrics result;
  if (overlap > math::kEpsilon) {
    grading::Eval(overlap, obstacle1, obstacle2,
                  &result.precision, &result.recall, &result.jaccard);
  }
  return result;
}

std::vector<int> PerceptionEvaluator::ComputeLidarPoints(
    const PointCloud& pointcloud, const Polygon2d& polygon) const {
  std::vector<int> points;
  const Rigid3d& lidar_pose = GetLidarPoseFromPointCloud(pointcloud);
  double min_x = std::numeric_limits<double>::infinity();
  double min_y = std::numeric_limits<double>::infinity();
  double max_x = -std::numeric_limits<double>::infinity();
  double max_y = -std::numeric_limits<double>::infinity();
  for (const Vec2d& point : polygon) {
    min_x = std::min(min_x, point.x - math::kEpsilon);
    min_y = std::min(min_y, point.y - math::kEpsilon);
    max_x = std::max(max_x, point.x + math::kEpsilon);
    max_y = std::max(max_y, point.y + math::kEpsilon);
  }
  for (int i = 0; i < pointcloud.points.size(); ++i) {
    const Eigen::Vector3d pt_world = lidar_pose * pointcloud.points[i];
    if (pt_world.x() >= min_x && pt_world.x() <= max_x &&
        pt_world.y() >= min_y && pt_world.y() <= max_y &&
        IsPointInOrOnBoundaryPolygon(polygon, {pt_world.x(), pt_world.y()})) {
      points.push_back(i);
    }
  }
  return points;
}
