// Copyright @2018 Pony AI Inc. All rights reserved.

#pragma once

#include "pnc/display/main_window.h"
#include "pnc/simulation/replay_system.h"

namespace display {

class ReplayMainWindow : public MainWindow {
 public:
  ReplayMainWindow(QWidget* parent, const std::string& simulation_history_path)
      : MainWindow(interface::simulation::SimulationConfig(), parent) {
    setWindowTitle("Replay");
    replay_system_ = std::make_unique<simulation::ReplaySystem>(simulation_history_path);
    replay_system_thread_ = std::thread([this]() {
      replay_system_->Initialize();
      replay_system_->Start();
    });
    SetupMenu();
  }

  virtual void timerEvent(QTimerEvent* /*event*/) override {
    data_ = replay_system_->FetchData();
    painter_widget_->set_simulation_system_data(data_);
    painter_widget_->update();
    std::unordered_map<std::string, interface::display::VariableViewList> variable_view_data;
    GetVariableViewList(&variable_view_data, data_);
    variable_dock_widget_->UpdateVariables(variable_view_data);
  }

 private:
  void SetupMenu() {
    QAction* action = nullptr;
    QMenu* menu = nullptr;
    // Play
    menu = menuBar()->addMenu(tr("&Play"));

    action = menu->addAction(tr("&Pause/Resume"));
    action->setShortcut(QKeySequence(Qt::Key_Space));
    connect(action, &QAction::triggered, this, [this] { replay_system_->PushPauseCommand(); });

    menu->addSeparator();

    action = menu->addAction(tr("&Half speed"));
    QList<QKeySequence> shortcuts;
    shortcuts << QKeySequence(Qt::Key_Minus) << QKeySequence(Qt::Key_Underscore);
    action->setShortcuts(shortcuts);
    connect(action, &QAction::triggered, this,
            [this] { replay_system_->PushSimulationSpeedCommand(0.5); });

    action = menu->addAction(tr("&Double speed"));
    shortcuts.clear();
    shortcuts << QKeySequence(Qt::Key_Plus) << QKeySequence(Qt::Key_Equal);
    action->setShortcuts(shortcuts);
    connect(action, &QAction::triggered, this,
            [this] { replay_system_->PushSimulationSpeedCommand(2.0); });
    menu->addSeparator();
    action = menu->addAction(tr("&Quit"));
    action->setShortcut(QKeySequence::Quit);
    connect(action, &QAction::triggered, this, [this] { QApplication::exit(0); });
  }

  std::unique_ptr<simulation::ReplaySystem> replay_system_;
  std::thread replay_system_thread_;

  interface::simulation::SimulationSystemData data_;
};
};
