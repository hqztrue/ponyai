#pragma once

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
//#include "homework6/display/main_window.h"
#include "homework6/map/map_lib.h"
#include "homework5/geometry/geometry.h"
#include "common/proto/route.pb.h"
//#include "common/utils/math/math_utils.h"
//#include<string>
//using namespace std;

//const std::string pony_root = "/home/hqz/ponyai/";
//const std::string pony_root = "/home/hqz/ponyai/";
const std::string pony_root = "/unsullied/sharefs/hqz/shared/hw/ponyai/";

void find_pred_succ();
double len2D(const interface::geometry::Vector3d& v);
double dist(const interface::geometry::Point2D& x, const interface::geometry::Point2D& y);
double CalcDistance(const interface::geometry::Vector3d& position, const interface::geometry::Point3D& destination);
double len(interface::route::Route &route);
void find_route(interface::route::Route &route);
void find_route(char path_src[], char path_dst[]);




