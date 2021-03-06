// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "homework6/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "homework6/simulation/vehicle_agent_factory.h"
#include "homework6/route/find_route.h"
#include "common/utils/math/transform/transform.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include<time.h>
#include<math.h>
#include<algorithm>
#include<vector>
#include<sys/time.h>
#include<unistd.h>

namespace hqztrue {

struct Timer{
	struct timeval start;
	Timer(){init();}
	void init(){gettimeofday(&start,NULL);}
	double time(){
		struct timeval end;
		gettimeofday(&end,NULL);
		long timeuse=1000000*(end.tv_sec-start.tv_sec)+end.tv_usec-start.tv_usec;
		return timeuse*1e-6;
	}
	void print(){printf("time=%.8lf\n",time());}
};

struct PID{
    double Kp, Ki, Kd, Pv, Iv, Dv, last_t, last_e, setpoint, windup_guard, output;
    PID(double _Kp=0, double _Ki=0, double _Kd=0):Kp(_Kp),Ki(_Ki),Kd(_Kd){init();}
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
	iter_num = 0;
	iter_time = 0.01;
	//controller.init();
	
	route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	route.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
	route.mutable_end_point()->set_x(agent_status.route_status().destination().x());
	route.mutable_end_point()->set_y(agent_status.route_status().destination().y());
	//interface::geometry::Point2D p;
	//p.set_x(agent_status.route_status().destination().x());
	//p.set_y(agent_status.route_status().destination().y());
	//route.set_end_point(p);
	find_route(route);
	route_point_id = 0;
	pid = PID(40, 2, 0.2);  //PID(100, 10, 1);
	pid_steer = PID(2, 0.5, 0.5);
	
	FILE *fout = fopen("/home/hqztrue/Desktop/v.txt", "w");
	fclose(fout);
  }
  
  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) {
	FILE *fout = fopen("/home/hqztrue/Desktop/v.txt", "a");
	fprintf(fout, "%.5lf %.5lf\n", iter_num * iter_time, len2D(agent_status.vehicle_status().velocity()));
	fclose(fout);
	
	Timer timer;
	++iter_num;
	if (agent_status.route_status().is_new_request()){
		
	}
	/*route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
	route.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
	route.mutable_end_point()->set_x(agent_status.route_status().destination().x());
	route.mutable_end_point()->set_y(agent_status.route_status().destination().y());
	find_route(route);*/
	interface::geometry::Vector3d rear_to_front_;
	rear_to_front_.set_x(vehicle_params().wheelbase());
	rear_to_front_.set_y(0);
	rear_to_front_.set_z(0);
	Eigen::Vector3d rear_to_front = math::transform::ToEigen(rear_to_front_);
	Eigen::Quaterniond trans = math::transform::ToEigen(agent_status.vehicle_status().orientation());
	Eigen::Vector3d rear_to_front_world_ = trans * rear_to_front;
	interface::geometry::Vector3d position = agent_status.vehicle_status().position(), rear_to_front_world = math::transform::ToProto(rear_to_front_world_);
	interface::geometry::Vector3d front_position;
	front_position.set_x(position.x() + rear_to_front_world.x());
	front_position.set_y(position.y() + rear_to_front_world.y());
	front_position.set_z(position.z() + rear_to_front_world.z());

	geometry::point p(front_position.x(), front_position.y()); //p(agent_status.vehicle_status().position().x(), agent_status.vehicle_status().position().y());
	double d_line = 1e10;
	while (1){
		geometry::point p1 = geometry::point(route.route_point(route_point_id).x(), route.route_point(route_point_id).y()),
		                p2 = geometry::point(route.route_point(route_point_id+1).x(), route.route_point(route_point_id+1).y()),
						p3 = (p2-p1).rotate(geometry::PI/2);
		geometry::line l(p2, p2+p3);
		if (route_point_id >= route.route_point_size()-2 || geometry::in_line(p, l)){
			geometry::point v1=p2-p1,v2=p-p1;
			d_line = (v1^v2)/v1.len();  //>0: left
			break;
		}
		++route_point_id;
	}
	
	//printf("find_route\n");timer.print();timer.init();
	
	//double dist = len(route);
	double dist = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination());
	double v_threshold = 5;
	double a_threshold = 0.5;
	double pos_threshold = 2.0;
    interface::control::ControlCommand command;
	double v = len2D(agent_status.vehicle_status().velocity());
	
	if (dist < pos_threshold){
		printf("finish\n");
		command.set_brake_ratio(1);
		return command;
	}
	else if (dist <= v*v/2/fabs(a_threshold)){
		double a = v*v/2/dist;
		/*double c = controller.query_c(v, -a);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		pid.set_setpoint(v-a*iter_time);
		pid.update(v, iter_num * iter_time);
	}
	else if (v<v_threshold){
		/*double c = controller.query_c(v, a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		pid.set_setpoint(v+a_threshold*iter_time);
		pid.update(v, iter_num * iter_time);
	}
	else if (v>v_threshold){
		/*double c = controller.query_c(v, -a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		pid.set_setpoint(v-a_threshold*iter_time);
		pid.update(v, iter_num * iter_time);
	}
	double u = pid.output;
	if (u>=0)command.set_throttle_ratio(u);
	else command.set_brake_ratio(-u);
	
	command.set_steering_rate(0);
	/*if (d_line>0){
		command.set_steering_angle(-5);
	}
	else {
		command.set_steering_angle(5);
	}*/
	
	pid_steer.set_setpoint(0);
	pid_steer.update(d_line, iter_num * iter_time);
	command.set_steering_angle(pid_steer.output);
	
    printf("%d %.5lf %.5lf %d/%d %.5lf\n",iter_num, v, u, route_point_id, route.route_point_size(), d_line);
	timer.print();
    return command;
  }

 private:

  interface::route::Route route;
  Controller controller;
  PID pid, pid_steer;
  int iter_num, route_point_id;
  double iter_time;
};

}

