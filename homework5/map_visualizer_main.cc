// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/display/main_window.h"
#include "homework5/map/map_lib.h"

DEFINE_string(route_file_path, "", "Path of displayed route");

bool point_equal(const Point3D &p1, const Point3D &p2){
	const double eps = 1e-3;
	return fabs(p1.x()-p2.x())<eps && fabs(p1.y()-p2.y())<eps && fabs(p1.z()-p2.z())<eps;
}

void find_pred_succ(){
	homework5::map::MapLib map_lib;
	interface::map::Map map = map_lib->map_proto();
	int n = map.lane_size();
	for (int i=0;i<n;++i){
		interface::map::Lane &lane1 = map.lane(i);
		for (int j=0;j<n;++j)
			if (i!=j){
				interface::map::Lane &lane2 = map.lane(j);
				//check lane(i)->lane(j), by central_line
				if (point_equal(lane1.central_line().point(lane1.central_line().point_size()-1),lane2.central_line().point(0))){
						interface::map::Id *pid = lane1.add_successor();
						pid->set_id(lane2.id());
						pid = lane2.add_predecessor();
						pid->set_id(lane1.id());
					}
			}
	}
	WriteProtoToTextFile(map, "processed_map_proto.txt");
}

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("pony.ai");
  QCoreApplication::setOrganizationDomain("pony.ai");
  QCoreApplication::setApplicationName("MapVisualizer");

  homework5::MainWindow main_window(nullptr);
  //find_pred_succ(); return 0;
  
  if (!FLAGS_route_file_path.empty()) {
    interface::route::Route route;
    CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &route)) << "Failed to load route file";
    main_window.set_displayed_route(route);
  }

  app.installEventFilter(&main_window);
  app.exec();

  return 0;
}
