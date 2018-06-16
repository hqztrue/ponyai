// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/display/main_window.h"
#include "homework5/map/map_lib.h"
#include "homework5/route/find_route.h"

DEFINE_string(route_file_path, "", "Path of displayed route");
DEFINE_string(map_dir, "", "Directory path of map file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);
  
  //find_pred_succ(); puts("pred_succ"); return 0;
  
  /*if (argc==1){
  for (int i=1;i<=5;++i){
	  char path_src[305], path_dst[305];
	  sprintf(path_src, "/home/hqztrue/Desktop/ponyai/homework5/data/routes/route_request_%d.txt", i);
	  sprintf(path_dst, "/home/hqztrue/Desktop/ponyai/homework5/data/routes/route_result_%d.txt", i);
	  find_route(path_src, path_dst);
  }
  puts("find_route"); return 0;
  */
  
  QApplication app(argc, argv);
  QCoreApplication::setOrganizationName("pony.ai");
  QCoreApplication::setOrganizationDomain("pony.ai");
  QCoreApplication::setApplicationName("MapVisualizer");

  homework5::MainWindow main_window(nullptr);
  
  if (!FLAGS_route_file_path.empty()) {
    interface::route::Route route;
    CHECK(file::ReadTextFileToProto(FLAGS_route_file_path, &route)) << "Failed to load route file";
    main_window.set_displayed_route(route);
  }

  app.installEventFilter(&main_window);
  app.exec();

  return 0;
}
