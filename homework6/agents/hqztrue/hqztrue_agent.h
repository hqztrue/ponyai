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

namespace hqztrue {

struct PID{
    double Kp, Ki, Kd, Pv, Iv, Dv, last_t, last_e, setpoint, windup_guard, output;
    PID(double _Kp, double _Ki, double _Kd):Kp(_Kp),Ki(_Ki),Kd(_Kd){init();}
    void init(){
        Pv = Iv = Dv = 0;
        last_t = 0;
        last_e = 0;
        setpoint = 0;
        windup_guard = 1;
        output = 0;
    }
    void update(double v, double t){
        double e = setpoint - v;
        Pv = e;
        Iv += e * (t-last_t);
        Iv = std::max(std::min(Iv, windup_guard), -windup_guard);
        Dv = (e-last_e) / (t-last_t);
        last_t = t;
        last_e = e;
        output = Kp * Pv + Ki * Iv + Kd * Dv;
    }
    void set_setpoint(double v){
        setpoint = v;
    }
    void set_windup_guard(double v){
        windup_guard = v;
    }
};
double update(double &y, double u, double t){
    //y+=(u-1./t);
    y=std::sin(u)+std::cos(u);
    //y=u*u*u;
    //y=u*u*u+sin(u)+cos(u)+u*u+1./u+0.1*y;
}
void test_PID(double P, double I, double D, int L){
    PID pid(P, I, D);
    double u = 0, y = 0;
    for (int i=1;i<=L;++i){
        pid.update(y, i);
        u = pid.output;
        if (pid.setpoint>0)update(y, u, i);
        printf("%.5lf %.5lf %d\n", y, u, i);
        if (i>=10)pid.set_setpoint(1);
    }
}


class FrogVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit FrogVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
    first_run = true;
	acceleration = true;
	control = delta_control = 0.1;
	
	route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	route.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
	route.mutable_end_point()->set_x(agent_status.route_status().destination().x());
	route.mutable_end_point()->set_y(agent_status.route_status().destination().y());
	//interface::geometry::Point2D p;
	//p.set_x(agent_status.route_status().destination().x());
	//p.set_y(agent_status.route_status().destination().y());
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

