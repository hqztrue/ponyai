// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "homework6/route/find_route.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include<time.h>
#include<math.h>
#include<algorithm>
#include<vector>

namespace table {

class TableVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit TableVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
    first_run = true;
	acceleration = true;
	output_flag = true;
	num_control = 10;
	control = delta_control = 1./num_control;
	//delta_v = 1e-10;
        delta_v = 0.05;
	prev_v = 0;
  }

  //generate table
  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
	
    const double max_velocity = 10, min_velocity = 0.1;
    interface::control::ControlCommand command;
    //double dist = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
	
	double v = len2D(agent_status.vehicle_status().velocity()), a = len2D(agent_status.vehicle_status().acceleration_vcs());
	if (first_run){
		FILE *f = fopen((pony_root+"homework6/table.txt").c_str(), "w");
		fclose(f);
	}
	else if (1||output_flag){
		FILE *f = fopen((pony_root+"homework6/table.txt").c_str(), "a");
		double dot = dot2D(prev_status.vehicle_status().velocity(), agent_status.vehicle_status().acceleration_vcs());
		//fprintf(f, "%.8lf %.8lf %.8lf\n",len2D(prev_status.vehicle_status().velocity()), prev_control, a*(dot>0?1:-1));
		fprintf(f, "%.5lf %.5lf %.5lf %d\n",len2D(prev_status.vehicle_status().velocity()), prev_control, a*(dot>0?1:-1), output_flag);
                fclose(f);
	}
	
	first_run = false;
	if (control > 1.0+geometry::eps)exit(0);
	//printf("control %.5lf %.5lf %.5lf\n",control, v, a);
	if (acceleration){
		if (fabs(v-prev_v)<delta_v){
			command.set_throttle_ratio(1);
			output_flag = false;
		}
		else {
			command.set_throttle_ratio(control);
			prev_control = control;
			output_flag = true;
			prev_v = v;
		}
		if (v>=max_velocity)acceleration = false;
	}
	else {
		if (fabs(v-prev_v)<delta_v){
			command.set_brake_ratio(1);
			output_flag = false;
		}
		else {
			command.set_brake_ratio(control);
			prev_control = -control;
			output_flag = true;
			prev_v = v;
		}
		if (v<=min_velocity){
			control += delta_control;
			acceleration = true;
		}
	}
	prev_status = agent_status;
    return command;
  }

 private:
  bool first_run, acceleration, output_flag;
  interface::agent::AgentStatus prev_status;
  double prev_control, control, delta_control, delta_v, prev_v;
  int num_control;
  interface::route::Route route;
};

}

