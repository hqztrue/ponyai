// Copyright @2018 Pony AI Inc. All rights reserved.

#include "pnc/simulation/metric_collector.h"

#include "pnc/utils/vehicle_status_helper.h"

namespace simulation {

void MetricCollector::AddMetricFrame(const interface::agent::AgentStatus& agent_status) {
  num_finished_trips_ = agent_status.simulation_status().num_finished_trips();
  if (!agent_status.simulation_status().is_alive()) {
    return;
  }
  time_cost_ = agent_status.simulation_status().simulation_time();
  if (agent_status.simulation_status().simulation_time() - last_frame_time_ <
      kDeltaT - math::kEpsilon) {
    return;
  }
  last_frame_time_ = agent_status.simulation_status().simulation_time();
  const interface::agent::VehicleStatus& vehicle_status = agent_status.vehicle_status();

  frame_count_++;
  if (frame_count_ == 1) {
    last_vehicle_status_.CopyFrom(vehicle_status);
    return;
  }

  double acc = vehicle_status.acceleration_vcs().x();
  if (std::abs(acc) > 3) {
    big_acc_sqr_sum_ += math::Sqr(acc);
  }
  if (frame_count_ >= 2) {
    math::Vec2d last_last_pos(last_last_vehicle_status_.position().x(),
                              last_last_vehicle_status_.position().y());
    math::Vec2d last_pos(last_vehicle_status_.position().x(), last_vehicle_status_.position().y());
    math::Vec2d current_pos(vehicle_status.position().x(), vehicle_status.position().y());
    curvature_sqr_sum_ += math::CurvatureSqr(last_last_pos, last_pos, current_pos);
  }
  last_last_vehicle_status_.Swap(&last_vehicle_status_);
  last_vehicle_status_.CopyFrom(vehicle_status);
}

interface::agent::AgentMetric MetricCollector::DumpMetric() {
  interface::agent::AgentMetric agent_metric;
  agent_metric.set_name(name_);
  agent_metric.set_num_finished_trips(num_finished_trips_);
  if (frame_count_ > 0) {
    agent_metric.set_big_acc_sqr_sum_total(big_acc_sqr_sum_);
    if (num_finished_trips_ > 0) {
      agent_metric.set_big_acc_sqr_sum_per_trip(big_acc_sqr_sum_ / num_finished_trips_);
    }
    agent_metric.set_curvature_sqr_sum_total(curvature_sqr_sum_);
    if (num_finished_trips_ > 0) {
      agent_metric.set_curvature_sqr_sum_per_trip(curvature_sqr_sum_ / num_finished_trips_);
    }
    agent_metric.set_time_cost_total(time_cost_);
    if (num_finished_trips_ > 0) {
      agent_metric.set_time_cost_per_trip(time_cost_ / num_finished_trips_);
    }
  }
  return agent_metric;
}

}  // namespace simulation
