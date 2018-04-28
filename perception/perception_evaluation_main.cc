// Copyright @2018 Pony AI Inc. All rights reserved.

#include <regex>

#include <gflags/gflags.h>
#include <glog/logging.h>

#include "common/proto/perception_evaluation.pb.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/strings/format.h"
#include "perception/perception_evaluator.h"

DECLARE_string(perception_evaluation_label_file_suffix);
DECLARE_string(perception_evaluation_data_file_suffix);

DEFINE_string(evaluation_config_dir, "", "Directory contains evaluation config files.");
DEFINE_string(result_folder, "", "Result folder.");

namespace {

constexpr double kEvaluationRange = 30.0;

std::unordered_map<std::string, std::string> ObtainDataToLabelMapping(
    const std::string& data_dir, const std::string& label_dir) {
  std::unordered_map<std::string, std::string> data_to_label_map;
  // Obtain all label files.
  std::vector<std::string> label_files =
      file::path::FindFilesWithPrefixSuffix(
          label_dir, "", FLAGS_perception_evaluation_label_file_suffix);
  LOG(INFO) << "Total label files found: " << label_files.size();

  // Construct data to label file map.
  for (auto const& label_file : label_files) {
    const int file_index = std::stoi(file::path::FilenameStem(label_file));
    const std::string data_file = file::path::Join(data_dir, strings::Format("{}.txt", file_index));
    if (file::path::Exists(data_file)) {
      data_to_label_map.emplace(data_file, label_file);
    } else {
      LOG(WARNING) << "Fail to find label for " << data_file;
    }
  }
  return data_to_label_map;
}

void RunPerceptionEvaluation() {
  if (FLAGS_result_folder.empty()) {
    FLAGS_result_folder = "/tmp/" + std::to_string(time(NULL)) + "/";
  }
  file::CreateFolder(FLAGS_result_folder);
  const auto config_files =
      file::path::FindFilesWithPrefixSuffix(FLAGS_evaluation_config_dir, "", ".config");

  double precision_sum = 0.0;
  double recall_sum = 0.0;
  int num_frames = 0;
  for (const auto& config_file : config_files) {
    interface::perception::PerceptionEvaluationConfig config;
    CHECK(file::ReadTextFileToProto(config_file, &config))
        << "Failed to load config file: " << config_file;
    const std::string result_file =
        file::path::Join(FLAGS_result_folder, strings::Format("{}.result", config.scenario_name()));
    const std::string lidar_dir = config.local_data().lidar_dir();
    const std::string label_dir = config.local_data().label_dir();
    const std::string camera_dir = config.local_data().camera_dir();
    CHECK(file::path::Exists(lidar_dir)) << lidar_dir << " doesn't exist.";
    CHECK(file::path::Exists(label_dir)) << label_dir << " doesn't exist.";;
    // Run perception evaluation.
    PerceptionEvaluator::Options options;
    options.scenario_name = config.scenario_name();
    options.lidar_folder = lidar_dir;
    options.camera_folder = camera_dir;
    options.data_to_label_map = ObtainDataToLabelMapping(lidar_dir, label_dir);
    options.evaluation_range = kEvaluationRange;
    auto evaluator = std::make_unique<PerceptionEvaluator>(options);
    const auto eval_result = evaluator->RunEvaluation();
    CHECK(file::WriteProtoToTextFile(eval_result, result_file));
    std::cout << "*********************************************************" << std::endl;
    std::cout << "Evaluation results for " << config_file << std::endl;
    for (const auto& frame_result : eval_result.frame_result()) {
      std::cout << frame_result.log_file_path()
                << ", [Precision:" << std::max(0.0, frame_result.precision())
                << "], [Recall: " << std::max(0.0, frame_result.recall()) << "]"
                << std::endl;
      precision_sum += frame_result.precision();
      recall_sum += frame_result.recall();
      ++num_frames;
    }
  }
  // Calculate the average precision and recall for all frames.
  double average_precision = 0.0;
  double average_recall = 0.0;
  if (num_frames != 0) {
    average_precision = precision_sum / num_frames;
    average_recall = recall_sum / num_frames;
  }
  std::cout << std::endl << std::endl;
  std::cout << "*********************************************************" << std::endl;
  std::cout << "* [Average Precision:" << std::setprecision(4) << average_precision
            << "], [Average Recall: " << average_recall
            << "] *" << std::endl;
  std::cout << "*********************************************************" << std::endl;
  std::cout << "Evaluation result folder: " << FLAGS_result_folder << std::endl;
}

}  // namespace

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  RunPerceptionEvaluation();
  return 0;
}
