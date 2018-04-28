// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "common/proto/simulation.pb.h"
#include "common/utils/common/mutex.h"
#include "common/utils/file/file.h"
#include "pnc/simulation/system_base.h"

namespace simulation {

class ReplaySystem : public SystemBase {
 public:
  struct SimulationStatus {
    bool playback_paused = false;
    double playback_speed_x = 1.0;
    bool playback_terminated = false;
  };

  explicit ReplaySystem(const std::string& record_filename) { record_filename_ = record_filename; }

  void Initialize();

  void Start();

  interface::simulation::SimulationSystemData FetchData() override;

 private:
  std::string record_filename_;

  utils::Mutex mutex_{"replay"};

  interface::simulation::SimulationSystemData simulation_frame_data_ GUARDED_BY(mutex_);
  std::unique_ptr<file::FileInterface> simulation_log_file_;
};

};  // namespace simulation
