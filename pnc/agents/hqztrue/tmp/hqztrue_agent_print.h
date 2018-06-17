// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/simulation/vehicle_agent.h"

#include "common/proto/agent_status.pb.h"
#include "common/proto/geometry.pb.h"
#include "common/proto/vehicle_status.pb.h"
#include "common/utils/math/math_utils.h"
#include "pnc/simulation/vehicle_agent_factory.h"
#include "pnc/agents/hqztrue/route/find_route.h"
#include "common/utils/math/transform/transform.h"
#include<stdio.h>
#include<stdlib.h>
#include<string.h>
#include<iostream>
#include<time.h>
#include<math.h>
#include<algorithm>
#include<vector>

namespace hqztrue {

constexpr int kNumIntervals = 4;
constexpr double kTimeInterval[4] = {20, 3, 20, 3};  //RYGY
constexpr double safe_time = 2.5, safe_dist = 5.0, safe_dist_low = 2.0, safe_radius = 5.0, safe_radius_car = 1.5;
  
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
	void print(){
		printf("%.5lf %.5lf %.5lf\n", Pv, Iv, Dv);
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
		FILE *fin = fopen((pony_root+"pnc/table.txt").c_str(), "r");
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

  virtual void Initialize(const interface::agent::AgentStatus& agent_status) override {
	init(agent_status, true);
  }
  int light_status(int id, double t, double cur_t){  //0:red 1:ok
	  double eps = 1e-6;
	  if (cur_t+eps<kTimeInterval[1]+iter_time)return t+eps<kTimeInterval[1]+iter_time?1:0;
	  else {
		  double total = kTimeInterval[0]+kTimeInterval[1]+kTimeInterval[2]+kTimeInterval[3];
		  double t1 = (t-kTimeInterval[1]) - floor((t-kTimeInterval[1])/total+eps)*total;
		  if (colors[id]==-1)return 1;
		  for (int i=0;i<4;++i){
			  int j = (colors[id]+i)%4;
			  double t2 = kTimeInterval[j];
			  if (t1<t2){
				  if (j==0)return 0;
				  if (j==1){
					  if (t1<safe_time)return 0;
					  else return 1;
				  }
				  if (j==2)return 1;
				  if (j==3){
					  if (t2-t1<safe_time)return 0;
					  else return 1;
				  }
			  }
			  t1 -= t2;
		  }
	  }
  }
  std::string to_lane_id(std::string s){
	  return s.substr(2, s.length()-5);
  }
  void init(const interface::agent::AgentStatus& agent_status, bool real_init=true){
	//printf("init%15.lf %15.lf %15.lf %15.lf\n",agent_status.vehicle_status().position().x(),agent_status.vehicle_status().position().y(),agent_status.route_status().destination().x(),agent_status.route_status().destination().y());
	//controller.init();
	
	//interface::geometry::Point2D p;
	//p.set_x(agent_status.route_status().destination().x());
	//p.set_y(agent_status.route_status().destination().y());
	//route.set_end_point(p);
	
	if (real_init){
		map = map_lib().map_proto();
		find_pred_succ(map);
	}
	else {
		iter_num = 0;
		iter_time = 0.01;
		v_setpoint_id = -1;
		v_setpoint_type = 0;
		v_setpoint = 0;
		
		route.mutable_start_point()->set_x(agent_status.vehicle_status().position().x());
		route.mutable_start_point()->set_y(agent_status.vehicle_status().position().y());
		route.mutable_end_point()->set_x(agent_status.route_status().destination().x());
		route.mutable_end_point()->set_y(agent_status.route_status().destination().y());
		find_route(route, map);
		route_point_id_front = 0;
		route_point_id = 0;
		
		//traffic light
		d_light = vector<double>(route.route_point_size()-1, 0);
		id_light = vector<int>(route.route_point_size()-1, -1);
		for (int i=route.route_point_size()-2;i>=0;--i){
			geometry::point p1 = geometry::point(route.route_point(i).x(), route.route_point(i).y()),
		                    p2 = geometry::point(route.route_point(i+1).x(), route.route_point(i+1).y());
			geometry::line l(p1, p2);
			for (int j=0;j<map.lane_size();++j){
				interface::geometry::Point3D q1 = map.lane(j).left_bound().boundary().point(map.lane(j).left_bound().boundary().point_size()-1),
											 q2 = map.lane(j).right_bound().boundary().point(map.lane(j).right_bound().boundary().point_size()-1);
				geometry::line l1(geometry::point(q1.x(),q1.y()), geometry::point(q2.x(),q2.y()));
				if (geometry::on_segment(p2, l1)){
					d_light[i] = 0;
					id_light[i] = j;
				}
			}
			
			if (i<route.route_point_size()-2 && id_light[i]==-1){
				geometry::point p3 = geometry::point(route.route_point(i+2).x(), route.route_point(i+2).y());
				id_light[i] = id_light[i+1];
				d_light[i] = d_light[i+1]+(p3-p2).len();
			}
		}
		pid = PID(0.5, 0.2, 0.002);
		pid_steer = PID(2, 0.5, 0.5);
	}
  }
  
  virtual interface::control::ControlCommand RunOneIteration(
      const interface::agent::AgentStatus& agent_status) override {
	//printf("iter%15.lf %15.lf %15.lf %15.lf\n",agent_status.vehicle_status().position().x(),agent_status.vehicle_status().position().y(),agent_status.route_status().destination().x(),agent_status.route_status().destination().y());
	puts("--------");
	Timer timer;
	if (agent_status.route_status().is_new_request()){
		init(agent_status, false);
	}
	++iter_num;
	PublishVariable("elimination_reason", std::string(agent_status.simulation_status().elimination_reason()), utils::display::Color::Red());
	
	double t = agent_status.simulation_status().simulation_time();
	//printf("init time: %.5lf %d\n",t, int(real_init));
	if (fabs(t-kTimeInterval[1]-iter_time)<1e-5){
		colors = vector<int>(map.lane_size(), -1);
		for (int i=0;i<agent_status.perception_status().traffic_light_size();++i){
			interface::perception::PerceptionTrafficLightStatus lights = agent_status.perception_status().traffic_light(i);
			for (int j=0;j<lights.single_traffic_light_status_size();++j){
				interface::perception::SingleTrafficLightStatus light = lights.single_traffic_light_status(j);
				for (int k=0;k<map.lane_size();++k)
					if (to_lane_id(light.id().id())==map.lane(k).id().id()){
						switch(light.color()){
							case interface::map::Bulb::RED: colors[k] = 0;
							break;
							case interface::map::Bulb::GREEN: colors[k] = 2;
							break;
							case interface::map::Bulb::YELLOW: puts("err");
							break;
							default:;
						}
					}
			}
		}
		//for (;;);
	}
	
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

	geometry::point p(front_position.x(), front_position.y()), p0(position.x(), position.y()), p_line_front, p_line_rear; //p(agent_status.vehicle_status().position().x(), agent_status.vehicle_status().position().y());
	double d_line = 1e10;
	//front
	while (1){
		geometry::point p1, p2, p3;
		p1 = geometry::point(route.route_point(route_point_id_front).x(), route.route_point(route_point_id_front).y());
		p_line_front = p2 = geometry::point(route.route_point(route_point_id_front+1).x(), route.route_point(route_point_id_front+1).y());
		p3 = (p2-p1).rotate(geometry::PI/2);
		geometry::line l(p2, p2+p3);
		if (route_point_id_front >= route.route_point_size()-2 || geometry::in_line(p, l)){
			geometry::point v1=p2-p1,v2=p-p1;
			d_line = (v1^v2)/v1.len();  //>0: left
			break;
		}
		++route_point_id_front;
	}
	//rear
	while (1){
		geometry::point p1, p2, p3;
		p1 = geometry::point(route.route_point(route_point_id).x(), route.route_point(route_point_id).y());
		p_line_rear = p2 = geometry::point(route.route_point(route_point_id+1).x(), route.route_point(route_point_id+1).y());
		p3 = (p2-p1).rotate(geometry::PI/2);
		geometry::line l(p2, p2+p3);
		if (route_point_id >= route.route_point_size()-2 || geometry::in_line(p0, l)){
			break;
		}
		++route_point_id;
	}
	
	printf("light, ");timer.print();timer.init();
	
	
	//double dist = len(route);
	double dist_end = CalcDistance(agent_status.vehicle_status().position(), agent_status.route_status().destination()), dist = dist_end;
	double v_threshold = 10, v_low_threshold = 0.3, v_hard_threshold = 50.0/3.6;
	double a_threshold = 2.5;
	double pos_threshold = 2.0;
    interface::control::ControlCommand command;
	double v = len2D(agent_status.vehicle_status().velocity());
	bool stop_flag = false;
	if (v > v_hard_threshold)stop_flag = true;
	
	//obstacles
	double stop_dist = v*v/2/fabs(a_threshold);
	for (int i=0;i<agent_status.perception_status().obstacle_size();++i){
		interface::perception::PerceptionObstacle obstacle = agent_status.perception_status().obstacle(i);
		if (obstacle.type()!=interface::perception::ObjectType::CAR && obstacle.type()!=interface::perception::ObjectType::PEDESTRIAN)continue;
		vector<geometry::point> pts;
		for (int j=0;j<obstacle.polygon_point_size();++j)
			pts.push_back(geometry::point(obstacle.polygon_point(j).x(), obstacle.polygon_point(j).y()));
		double max_velocity, safe_r;
		if (obstacle.type()==interface::perception::ObjectType::CAR){
			max_velocity = 55.0/3.6;
			safe_r = safe_radius_car;
		}
		else if (obstacle.type()==interface::perception::ObjectType::PEDESTRIAN){
			max_velocity = 6.0/3.6;
			safe_r = safe_radius;
		}
		double d = (p_line_front-p).len() - vehicle_params().vehicle_fa_to_front();
		for (int j=route_point_id_front+1;j<route.route_point_size();++j){
			geometry::point q = geometry::point(route.route_point(j).x(), route.route_point(j).y()),
			                q_ = geometry::point(route.route_point(j-1).x(), route.route_point(j-1).y());
			if (j>route_point_id_front+1)d += (q-q_).len();
			if (d>stop_dist + safe_dist)break;
			double t_ = std::min((d+vehicle_params().vehicle_length())/std::max(0.01, v), v/a_threshold), dmin = 1e10;
			for (int k=0;k<pts.size();++k)
				dmin = std::min(dmin, (pts[k]-q).len());
			if (dmin<max_velocity*t_+safe_r){
				stop_flag = true;
			}
		}
	}
	
	//steer
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
	
	printf("test light\n");
	if (id_light[route_point_id]!=-1){
		double d = d_light[route_point_id];
		d += (p_line_rear-p0).len();  //- vehicle_params().vehicle_fa_to_front();
		//d = std::max(d, 0.0);
		double t1 = 0;
		t1 = fabs(v_threshold-v)/a_threshold;
		double x = t1*(v_threshold+v)/2;
		if (x<=d)t1+=(d-x)/v_threshold;
		else {
			if (v>=v_threshold)t1 = (sqrt(std::max(v*v+2*a_threshold*d,0.0))-v)/a_threshold;
			else t1 = (v-sqrt(std::max(v*v-2*a_threshold*d,0.0)))/a_threshold;
		}
		printf("t=%.5lf t1=%.5lf\n",t, t1);
		if (light_status(id_light[route_point_id], t+t1, t)==0){
			dist = std::min(dist, d);
			//dist = std::min(dist, d);
			printf("d=%.5lf\n",d);
		}
	}
	
	printf("control\n");
	//velocity
	if (dist_end < pos_threshold){
		printf("finish\n");
		command.set_brake_ratio(1);
		return command;
	}
	else if (v>v_low_threshold && dist <= v*v/2/fabs(a_threshold)+safe_dist || v<=v_low_threshold && dist <= safe_dist_low){
		double a = v*v/2/(std::max(dist,0.01));
		/*double c = controller.query_c(v, -a);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		printf("a=%.5lf\n",a);
		if (v_setpoint_id!=iter_num-1 || v_setpoint_type!=0)v_setpoint = v;
		v_setpoint_id = iter_num;
		v_setpoint_type = 0;
		v_setpoint = std::max(0.0, v_setpoint-a_threshold*iter_time);
		pid.set_setpoint(v_setpoint);
		pid.update(v, iter_num * iter_time);
		pid.output = -1;
	}
	else if (v>v_threshold || stop_flag){
		/*double c = controller.query_c(v, -a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		if (v_setpoint_id!=iter_num-1 || v_setpoint_type!=2)v_setpoint = v;
		v_setpoint_id = iter_num;
		v_setpoint_type = 2;
		v_setpoint = std::max(0.0, v_setpoint-a_threshold*iter_time);
		pid.set_setpoint(v_setpoint);
		pid.update(v, iter_num * iter_time);
		if (stop_flag)pid.output = -1;
	}
	else if (v<v_threshold){
		/*double c = controller.query_c(v, a_threshold);
		if (c>=0)command.set_throttle_ratio(c);
		else command.set_brake_ratio(-c);*/
		if (v_setpoint_id!=iter_num-1 || v_setpoint_type!=1)v_setpoint = v;
		v_setpoint_id = iter_num;
		v_setpoint_type = 1;
		v_setpoint = std::min(v_threshold, v_setpoint+a_threshold*iter_time);
		pid.set_setpoint(v_setpoint);
		pid.update(v, iter_num * iter_time);
	}
	double u = pid.output;
	if (u>=0)command.set_throttle_ratio(std::min(u, 1.0));
	else command.set_brake_ratio(std::max(-1.0, -u));
	
	pid.print();
    printf("%d v=%.5lf u=%.5lf sp=%.5lf %d(%d)/%d dist=%.5lf d_line=%.5lf stop_flag=%d\n",iter_num, v, u, pid.setpoint, route_point_id, route_point_id_front, route.route_point_size(), dist, d_line, int(stop_flag));
	printf("total, ");timer.print();
    return command;
  }

 private:

  interface::map::Map map;
  interface::route::Route route;
  Controller controller;
  PID pid, pid_steer;
  int iter_num, route_point_id, route_point_id_front, v_setpoint_id, v_setpoint_type;
  double iter_time, v_setpoint;
  vector<double> d_light;
  vector<int> id_light;
  vector<int> colors;  //interface::map::Bulb::Color
};

}

