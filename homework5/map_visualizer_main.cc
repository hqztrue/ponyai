// Copyright @2018 Pony AI Inc. All rights reserved.

#include "glog/logging.h"
#include "common/utils/file/file.h"
#include "gflags/gflags.h"
#include "homework5/display/main_window.h"
#include "homework5/map/map_lib.h"

DEFINE_string(route_file_path, "", "Path of displayed route");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

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
