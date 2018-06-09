#pragma once

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
//#include "homework6/display/main_window.h"
#include "homework6/map/map_lib.h"
#include "homework5/geometry/geometry.h"
#include "common/proto/route.pb.h"
//#include<string>
//using namespace std;

//const std::string pony_root = "/home/hqz/ponyai/";
//const std::string pony_root = "/home/hqz/ponyai/";
const std::string pony_root = "/unsullied/sharefs/hqz/shared/hw/ponyai/";

void find_pred_succ();
double len(interface::route::Route &route);
void find_route(interface::route::Route &route);
void find_route(char path_src[], char path_dst[]);




