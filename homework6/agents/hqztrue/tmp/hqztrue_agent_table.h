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

struct Controller{
	struct item{
		item(double v_=0, double c_=0, double a_=0):v(v_),c(c_),a(a_){}
		double v,c,a;
		bool operator <(const item &p)const{
			return v<p.v;
		}
	};
	std::vector<std::vector<item> > V;
	int num_control;
	double get_c(int i){
		return 1.0/num_control*(i-num_control);
	}
	void init(){
		num_control = 10;
		V.resize(num_control*2+1);
		for (int i=0;i<V.size();++i)
			V[i].clear();
		FILE *fin = fopen((pony_root+"homework6/table.txt").c_str(), "r");
		assert(fin!=NULL);
		double v, c, a;
		while (fscanf(fin, "%lf%lf%lf",&v,&c,&a)!=EOF){
			for (int i=0;i<V.size();++i)
				if (std::fabs(get_c(i)-c)<geometry::eps)
					V[i].push_back(item(v,c,a));
		}
		fclose(fin);
		for (int i=0;i<V.size();++i)std::sort(V[i].begin(),V[i].end());
	}
	double query_c(double v, double a){
		double ans_c = 0, err = 1e10, err1, errd;
		for (int i=0;i<V.size();++i)
			if (V[i].size()>0){
				std::vector<item>::iterator vit = std::lower_bound(V[i].begin(), V[i].end(), item(v,0,0));
				if (vit!=V[i].end()){
					err1=fabs(a-vit->a);
					errd=vit->v-v;
					if (vit!=V[i].begin()){
						--vit;
						if (v-vit->v<errd)err1=fabs(a-vit->a);
					}
				}
				else {
					--vit;
					err1=fabs(a-vit->a);
				}
				if (err1<err)err=err1, ans_c=get_c(i);
				
			}
		return ans_c;
	}
	double query_d(double v, double a){
		return v*v/2/fabs(a);
	}
};

class FrogVehicleAgent : public simulation::VehicleAgent {
 public:
  explicit FrogVehicleAgent(const std::string& name) : VehicleAgent(name) {}

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) {
	controller.init();
	  
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
  
  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
	if (agent_status.route_status().is_new_request()){
		
	}
	route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	route.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
	route.mutable_end_point()->set_x(agent_status.route_status().destination().x());
	route.mutable_end_point()->set_y(agent_status.route_status().destination().y());
	find_route(route);
	
	//double dist = len(route);
	double dist = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
	double v_threshold = 5;
	double a_threshold = 0.5;
	double pos_threshold = 3.0;
    interface::control::ControlCommand command;
	double v = len2D(agent_status.vehicle_status().velocity());
	
	if (dist < pos_threshold){
		printf("finish\n");
	}
	else if (dist <= controller.query_d(v, -a_threshold)){
		double a = v*v/2/dist;
		double c = controller.query_c(v, -a);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(c);
	}
	else if (v<v_threshold){
		double c = controller.query_c(v, a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(c);
	}
	else if (v>v_threshold){
		double c = controller.query_c(v, -a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(c);
	}
    return command;
  }

 private:

  interface::route::Route route;
  Controller controller;
};

}

