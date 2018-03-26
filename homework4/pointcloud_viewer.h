// Copyright @2018 Pony AI Inc. All rights reserved.

#include <gflags/gflags.h>
#include <glog/logging.h>
#include <QtWidgets/QApplication>

#include "common/proto/object_labeling_3d.pb.h"
#include "common/utils/display/painter_widget_base.h"
#include "common/utils/file/file.h"
#include "common/utils/file/path.h"
#include "common/utils/strings/format.h"

#include "homework4/camera_lidar_fusion_utils.h"

class PointCloudViewer : public utils::display::PainterWidgetBase {
 public:
  struct PointCloudLabel {
    std::string id;
    std::vector<math::Vec2d> polygon;
    double floor = std::numeric_limits<double>::infinity();
    double ceiling = -std::numeric_limits<double>::infinity();
  };

  using Options = utils::display::PainterWidgetBase::Options;

  PointCloudViewer(Options options, QWidget* parent, const std::string& data_dir);

  ~PointCloudViewer() override = default;

 protected:
  // Qt event handlers.
  void timerEvent(QTimerEvent* /*event*/) override {
    update();
  }

  utils::display::OpenglPainter* gl_painter() override {
    return gl_painter_.get();
  }

  void Initialize() override {};

  void InitializeGlPainter() override;

  void keyPressEvent(QKeyEvent* event) override;

  void Paint3D() override;

 private:
  std::unordered_map<std::string, std::string> ObtainDataToLabelMapping(
      const std::string& data_dir, const std::string& label_dir);

  void DrawPointCloudLabel(const PointCloudLabel& label);

  std::string data_dir_;
  std::vector<std::string> pointcloud_files_;
  std::unordered_map<std::string, std::string> data_label_map_;
  int file_index_ = -1;
  std::vector<math::Vec3d> points_;
  std::vector<PointCloudLabel> labels_;

  std::unique_ptr<utils::display::OpenglPainter> gl_painter_;
  utils::display::OpenglPainter::SurfaceStyle default_prism_style_;
  utils::display::OpenglPainter::PointStyle default_point_style_;

  DISALLOW_COPY_MOVE_AND_ASSIGN(PointCloudViewer);
};

