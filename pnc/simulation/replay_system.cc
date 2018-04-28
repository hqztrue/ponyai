// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/simulation/replay_system.h"
#include <thread>

namespace simulation {

void ReplaySystem::Initialize() {
  simulation_log_file_ = std::make_unique<file::File>(record_filename_, "r");
}

void ReplaySystem::Start() {
  auto system_time = std::chrono::steady_clock::now();
  while (1) {
    FlushPlaybackCommands();
    if (playback_status().playback_paused) {
      std::this_thread::sleep_for(std::chrono::milliseconds(10));
      continue;
    }
    {
      utils::MutexLocker lock(&mutex_);
      // simulation_frame_data_.CopyFrom(simulation_history_data_.simulation_frame(i));
      uint64_t size = 0;
      std::string content;
      if (simulation_log_file_->Read(&(size), sizeof(size)) == 0) {
        break;
      }
      content.resize(size);
      CHECK(simulation_log_file_->Read(&(content)[0], size) == size);
      simulation_frame_data_.ParseFromString(content);
    }
    system_time += std::chrono::microseconds(
        static_cast<int64_t>(10000.0 / playback_status().playback_speed_x));
    std::this_thread::sleep_until(system_time);
  }
}

interface::simulation::SimulationSystemData ReplaySystem::FetchData() {
  interface::simulation::SimulationSystemData result;
  {
    utils::MutexLocker lock(&mutex_);
    result.CopyFrom(simulation_frame_data_);
  }
  return result;
}
}
