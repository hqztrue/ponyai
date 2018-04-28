// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "common/proto/perception.pb.h"
#include "common/utils/common/defines.h"
#include "common/utils/common/optional.h"
#include "homework2/pointcloud.h"

class Perception {
 public:
  Perception() = default;

  interface::perception::PerceptionObstacles RunPerception(const PointCloud& pointcloud,
                                                           const utils::Optional<cv::Mat>& image);

  DISALLOW_COPY_MOVE_AND_ASSIGN(Perception);
};

