#pragma once

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/map/map_lib.h"
#include "homework5/route/geometry.h"
//#include<string>
//using namespace std;

void find_route(interface::route::Route &route);
void find_route(char path_src[], char path_dst[]);




