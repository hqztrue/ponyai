// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "homework6/route/find_route.h"

namespace hqztrue {

class FrogVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit FrogVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
    first_run = true;
	acceleration = true;
	control = delta_control = 0.1;
	
	interface::geometry::Point2D p;
	//route.start_point().set_x(agent_status.vehicle_status().position().x());
	//route.start_point().set_y(agent_status.vehicle_status().position().y());
	route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	p.set_x(agent_status.route_status().destination().x());
	p.set_y(agent_status.route_status().destination().y());
	//route.set_end_point(p);
	find_route(route);
  }

  //generate table
  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
    const double max_velocity = 10, eps = 1e-5;
    interface::control::ControlCommand command;
    double dist = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
	
	if (control > 1.0+eps)exit(0);
	if (acceleration){
		command.set_throttle_ratio(control);
		prev_control = control;
		if (len(agent_status.vehicle_status().velocity())>=max_velocity)acceleration = false;
	}
	else {
		command.set_brake_ratio(control);
		prev_control = -control;
		if (len(agent_status.vehicle_status().velocity())<=0.1){
			control += delta_control;
			acceleration = true;
		}
	}
	
	
	if (!first_run){
		FILE *f = fopen("/home/hqz/ponyai/homework6/table.txt", "a");
		fprintf(f, "%.6lf %.6lf %.6lf\n",len(prev_status.vehicle_status().velocity()), prev_control, len(agent_status.vehicle_status().acceleration_vcs()));
		fclose(f);
	}
	first_run = false;
	prev_status = agent_status;
    return command;
  }
  
  virtual interface::control::ControlCommand RunOneIteration1(
      const interface::agent::AgentStatus& agent_status) {
	const double max_velocity = 10, eps = 1e-5;
	double velocity_threshold = 5;
    interface::control::ControlCommand command;
    double dist = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
	
	if (dist<0){
		
	}
	else if (len(agent_status.vehicle_status().velocity())<velocity_threshold){
		command.set_throttle_ratio(0.3);
	}
	else if (len(agent_status.vehicle_status().velocity())>velocity_threshold){
		command.set_brake_ratio(0.3);
	}
    return command;
  }

 private:
  double CalcDistance(const interface::geometry::Vector3d& position,
                      const interface::geometry::Point3D& destination) {
    double sqr_sum =
        math::Sqr(position.x() - destination.x()) + math::Sqr(position.y() - destination.y());
    ;
    return std::sqrt(sqr_sum);
  }

  double len(const interface::geometry::Vector3d& v) {  //x, y
    return std::sqrt(math::Sqr(v.x()) + math::Sqr(v.y()));
  }
  bool first_run, acceleration;
  interface::agent::AgentStatus prev_status;
  double prev_control, control, delta_control;
  interface::route::Route route;
};

}

