// Copyright @2018 Pony AI Inc. All rights reserved.

#include <QtWidgets/QApplication>

#include "perception/evaluation_viewer.h"

DEFINE_string(eval_result_file, "", "Evaluation result file");

int main(int argc, char* argv[]) {
  google::ParseCommandLineFlags(&argc, &argv, true);
  google::InitGoogleLogging(argv[0]);

  QApplication app(argc, argv);
  EvaluationViewer::Options options;
  EvaluationViewer viewer(options, nullptr, FLAGS_eval_result_file);
  viewer.resize(1280, 960);
  viewer.show();
  app.exec();
  return 0;
}
