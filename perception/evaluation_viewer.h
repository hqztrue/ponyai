// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include <QtWidgets/QApplication>

#include "common/utils/display/painter_widget_base.h"
#include "common/utils/file/path.h"
#include "common/utils/strings/format.h"
#include "perception/perception_evaluator.h"

class EvaluationViewer : public utils::display::PainterWidgetBase {
 public:
  using Options = utils::display::PainterWidgetBase::Options;

  EvaluationViewer(Options options, QWidget* parent, const std::string& eval_result_file);

  ~EvaluationViewer() override = default;

 protected:
  void Initialize() override {};

  void InitializeGlPainter() override;

  utils::display::OpenglPainter* gl_painter() override {
    return gl_painter_.get();
  }

  // Qt event handlers.
  void timerEvent(QTimerEvent* /*event*/) override {
    update();
  }

  void keyPressEvent(QKeyEvent* event) override;

  void Paint3D() override;

 private:
  struct PolygonInfo {
    std::string id;
    double floor = 0.0;
    double ceiling = 0.0;
    std::string type;
    std::vector<math::Vec2d> polygon;
  };

  void ParsePerceptionFrameResult(const interface::perception::PerceptionFrameResult& eval_result);

  PolygonInfo ParseEffectivePolygonInfo(
      const interface::perception::EffectivePolygonInfo& polygon_info_pb);

  void DrawPolygonInfo(const std::vector<PolygonInfo>& polygon_info_vector,
                       const utils::display::Color& color);

  std::unique_ptr<utils::display::OpenglPainter> gl_painter_;
  utils::display::OpenglPainter::SurfaceStyle default_prism_style_;
  utils::display::OpenglPainter::PointStyle default_point_style_;
  bool show_detected_polygon_ = true;
  bool show_labled_polygon_ = true;

  interface::perception::PerceptionEvaluationResult eval_result_;
  int frame_index_ = -1;
  std::vector<math::Vec3d> points_;
  std::vector<PolygonInfo> labeled_polygon_;
  std::vector<PolygonInfo> detected_polygon_;

  DISALLOW_COPY_MOVE_AND_ASSIGN(EvaluationViewer);
};
