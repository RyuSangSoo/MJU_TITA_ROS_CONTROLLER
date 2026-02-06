#include "tita_ui/ui/ImuJointWidget.hpp"

#include <QAbstractItemView>
#include <QGridLayout>
#include <QGroupBox>
#include <QHeaderView>
#include <QLabel>
#include <QScrollArea>
#include <QTableWidgetItem>
#include <QVBoxLayout>

#include "tita_ui/ui/SignalPlotWidget.hpp"

QString ImuJointWidget::formatDouble(double value)
{
  return QString::number(value, 'f', 6);
}

ImuJointWidget::ImuJointWidget(QWidget *parent)
  : QWidget(parent)
{
  setStyleSheet(
    "QWidget { font-family: 'Noto Sans'; font-size: 15px; }"
    "QGroupBox { font-weight: 600; border: 1px solid #c9c9c9; border-radius: 6px; margin-top: 14px; }"
    "QGroupBox::title { subcontrol-origin: margin; left: 10px; padding: 0 4px; }"
    "QTableWidget { gridline-color: #d7d7d7; }"
    "QHeaderView::section { background: #f0f0f0; padding: 6px; border: 0px; }"
  );

  auto *root_layout = new QVBoxLayout(this);
  root_layout->setContentsMargins(12, 12, 12, 12);
  root_layout->setSpacing(12);

  auto *graphs_group = new QGroupBox("Graphs", this);
  auto *graphs_layout = new QVBoxLayout(graphs_group);

  auto *scroll = new QScrollArea(graphs_group);
  scroll->setWidgetResizable(true);

  auto *scroll_widget = new QWidget(scroll);
  auto *graphs_stack = new QVBoxLayout(scroll_widget);
  graphs_stack->setContentsMargins(8, 8, 8, 8);
  graphs_stack->setSpacing(12);

  auto add_plot = [&](QGridLayout *layout, const QString &name,
                      std::function<double(const TitaState &)> getter,
                      int columns)
  {
    auto *cell = new QWidget(scroll_widget);
    auto *cell_layout = new QVBoxLayout(cell);
    cell_layout->setContentsMargins(0, 0, 0, 0);
    cell_layout->setSpacing(6);

    auto *label = new QLabel(name, cell);
    auto *plot = new SignalPlotWidget(cell);
    plot->setTimeWindow(10.0);

    cell_layout->addWidget(label);
    cell_layout->addWidget(plot);

    const int index = layout->count();
    const int row = index / columns;
    const int col = index % columns;
    layout->addWidget(cell, row, col);

    plots_.push_back({name, plot, std::move(getter)});
  };

  auto *imu_group = new QGroupBox("Imu", scroll_widget);
  auto *imu_layout = new QGridLayout(imu_group);
  imu_layout->setHorizontalSpacing(12);
  imu_layout->setVerticalSpacing(12);
  add_plot(imu_layout, "roll", [](const TitaState &s) { return s.roll; }, 3);
  add_plot(imu_layout, "pitch", [](const TitaState &s) { return s.pitch; }, 3);
  add_plot(imu_layout, "yaw", [](const TitaState &s) { return s.yaw; }, 3);

  auto *j1_group = new QGroupBox("Joint 1", scroll_widget);
  auto *j1_layout = new QGridLayout(j1_group);
  j1_layout->setHorizontalSpacing(12);
  j1_layout->setVerticalSpacing(12);
  add_plot(j1_layout, "Left.J1", [](const TitaState &s) { return s.Left.Pos.joint1; }, 2);
  add_plot(j1_layout, "Right.J1", [](const TitaState &s) { return s.Right.Pos.joint1; }, 2);

  auto *j2_group = new QGroupBox("Joint 2", scroll_widget);
  auto *j2_layout = new QGridLayout(j2_group);
  j2_layout->setHorizontalSpacing(12);
  j2_layout->setVerticalSpacing(12);
  add_plot(j2_layout, "Left.J2", [](const TitaState &s) { return s.Left.Pos.joint2; }, 2);
  add_plot(j2_layout, "Right.J2", [](const TitaState &s) { return s.Right.Pos.joint2; }, 2);

  auto *j3_group = new QGroupBox("Joint 3", scroll_widget);
  auto *j3_layout = new QGridLayout(j3_group);
  j3_layout->setHorizontalSpacing(12);
  j3_layout->setVerticalSpacing(12);
  add_plot(j3_layout, "Left.J3", [](const TitaState &s) { return s.Left.Pos.joint3; }, 2);
  add_plot(j3_layout, "Right.J3", [](const TitaState &s) { return s.Right.Pos.joint3; }, 2);

  graphs_stack->addWidget(imu_group);
  graphs_stack->addWidget(j1_group);
  graphs_stack->addWidget(j2_group);
  graphs_stack->addWidget(j3_group);
  graphs_stack->addStretch(1);

  scroll_widget->setLayout(graphs_stack);
  scroll->setWidget(scroll_widget);
  graphs_layout->addWidget(scroll);

  auto *tita_group = new QGroupBox("Tita State", this);
  auto *tita_layout = new QVBoxLayout(tita_group);

  tita_roll_ = new QLabel("roll: -", tita_group);
  tita_pitch_ = new QLabel("pitch: -", tita_group);
  tita_yaw_ = new QLabel("yaw: -", tita_group);
  log_button_ = new QPushButton("Start Logging (R)", tita_group);
  connect(log_button_, &QPushButton::clicked, this, &ImuJointWidget::toggleLogging);
  log_status_ = new QLabel("log: idle", tita_group);

  tita_left_table_ = new QTableWidget(3, 4, tita_group);
  tita_left_table_->setHorizontalHeaderLabels({"J1", "J2", "J3", "Wheel"});
  tita_left_table_->setVerticalHeaderLabels({"Pos", "Vel", "Eff"});
  tita_left_table_->horizontalHeader()->setStretchLastSection(true);
  tita_left_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tita_left_table_->setSelectionMode(QAbstractItemView::NoSelection);

  tita_right_table_ = new QTableWidget(3, 4, tita_group);
  tita_right_table_->setHorizontalHeaderLabels({"J1", "J2", "J3", "Wheel"});
  tita_right_table_->setVerticalHeaderLabels({"Pos", "Vel", "Eff"});
  tita_right_table_->horizontalHeader()->setStretchLastSection(true);
  tita_right_table_->setEditTriggers(QAbstractItemView::NoEditTriggers);
  tita_right_table_->setSelectionMode(QAbstractItemView::NoSelection);

  tita_layout->addWidget(tita_roll_);
  tita_layout->addWidget(tita_pitch_);
  tita_layout->addWidget(tita_yaw_);
  tita_layout->addWidget(log_button_);
  tita_layout->addWidget(log_status_);
  tita_layout->addWidget(new QLabel("Left", tita_group));
  tita_layout->addWidget(tita_left_table_);
  tita_layout->addWidget(new QLabel("Right", tita_group));
  tita_layout->addWidget(tita_right_table_);

  root_layout->addWidget(graphs_group, 2);
  root_layout->addWidget(tita_group, 1);

  timer_.start();

  refresh_timer_ = new QTimer(this);
  refresh_timer_->setInterval(33);
  connect(refresh_timer_, &QTimer::timeout, this, &ImuJointWidget::refreshUi);
  refresh_timer_->start();
}

void ImuJointWidget::keyPressEvent(QKeyEvent *event)
{
  if (event->key() == Qt::Key_Escape)
  {
    close();
    return;
  }
  if (event->key() == Qt::Key_R)
  {
    toggleLogging();
    return;
  }

  QWidget::keyPressEvent(event);
}

void ImuJointWidget::onTitaStateUpdated(const TitaState &state)
{
  latest_state_ = state;
  has_state_ = true;
}

void ImuJointWidget::refreshUi()
{
  if (!has_state_)
  {
    return;
  }

  const TitaState &state = latest_state_;

  tita_roll_->setText(QString("roll: %1").arg(formatDouble(state.roll)));
  tita_pitch_->setText(QString("pitch: %1").arg(formatDouble(state.pitch)));
  tita_yaw_->setText(QString("yaw: %1").arg(formatDouble(state.yaw)));

  auto set_cell = [](QTableWidget *table, int row, int col, double value)
  {
    QTableWidgetItem *item = table->item(row, col);
    if (!item)
    {
      item = new QTableWidgetItem();
      table->setItem(row, col, item);
    }
    item->setText(QString::number(value, 'f', 6));
  };

  set_cell(tita_left_table_, 0, 0, state.Left.Pos.joint1);
  set_cell(tita_left_table_, 0, 1, state.Left.Pos.joint2);
  set_cell(tita_left_table_, 0, 2, state.Left.Pos.joint3);
  set_cell(tita_left_table_, 0, 3, state.Left.Pos.Wheel);
  set_cell(tita_left_table_, 1, 0, state.Left.Vel.joint1);
  set_cell(tita_left_table_, 1, 1, state.Left.Vel.joint2);
  set_cell(tita_left_table_, 1, 2, state.Left.Vel.joint3);
  set_cell(tita_left_table_, 1, 3, state.Left.Vel.Wheel);
  set_cell(tita_left_table_, 2, 0, state.Left.Effort.joint1);
  set_cell(tita_left_table_, 2, 1, state.Left.Effort.joint2);
  set_cell(tita_left_table_, 2, 2, state.Left.Effort.joint3);
  set_cell(tita_left_table_, 2, 3, state.Left.Effort.Wheel);

  set_cell(tita_right_table_, 0, 0, state.Right.Pos.joint1);
  set_cell(tita_right_table_, 0, 1, state.Right.Pos.joint2);
  set_cell(tita_right_table_, 0, 2, state.Right.Pos.joint3);
  set_cell(tita_right_table_, 0, 3, state.Right.Pos.Wheel);
  set_cell(tita_right_table_, 1, 0, state.Right.Vel.joint1);
  set_cell(tita_right_table_, 1, 1, state.Right.Vel.joint2);
  set_cell(tita_right_table_, 1, 2, state.Right.Vel.joint3);
  set_cell(tita_right_table_, 1, 3, state.Right.Vel.Wheel);
  set_cell(tita_right_table_, 2, 0, state.Right.Effort.joint1);
  set_cell(tita_right_table_, 2, 1, state.Right.Effort.joint2);
  set_cell(tita_right_table_, 2, 2, state.Right.Effort.joint3);
  set_cell(tita_right_table_, 2, 3, state.Right.Effort.Wheel);

  const double t = timer_.elapsed() / 1000.0;
  for (auto &binding : plots_)
  {
    binding.plot->appendSample(t, binding.getter(state));
  }
}

void ImuJointWidget::toggleLogging()
{
  if (!logging_enabled_)
  {
    QDir base_dir(QDir::currentPath());
    if (!base_dir.exists("data"))
    {
      base_dir.mkpath("data");
    }
    QDir data_dir = QDir(base_dir.filePath("data"));

    if (!data_dir.exists() || !data_dir.isReadable() || !QFileInfo(data_dir.absolutePath()).isWritable())
    {
      const QString home = QStandardPaths::writableLocation(QStandardPaths::HomeLocation);
      base_dir = QDir(home);
      base_dir.mkpath("tita_logs");
      data_dir = QDir(base_dir.filePath("tita_logs"));
    }

    const QString timestamp = QDateTime::currentDateTime().toString("yyyyMMdd_HHmmss");
    const QString file_path = data_dir.filePath(QString("tita_%1.csv").arg(timestamp));

    log_file_.setFileName(file_path);
    if (!log_file_.open(QIODevice::WriteOnly | QIODevice::Text))
    {
      if (log_status_)
      {
        log_status_->setText(QString("log: failed (%1)").arg(file_path));
      }
      return;
    }

    log_stream_.setDevice(&log_file_);
    log_stream_ << "time_sec;roll;pitch;yaw;"
                << "left_pos_j1;left_pos_j2;left_pos_j3;left_pos_wheel;"
                << "left_vel_j1;left_vel_j2;left_vel_j3;left_vel_wheel;"
                << "left_eff_j1;left_eff_j2;left_eff_j3;left_eff_wheel;"
                << "right_pos_j1;right_pos_j2;right_pos_j3;right_pos_wheel;"
                << "right_vel_j1;right_vel_j2;right_vel_j3;right_vel_wheel;"
                << "right_eff_j1;right_eff_j2;right_eff_j3;right_eff_wheel\n";

    logging_enabled_ = true;
    if (log_button_)
    {
      log_button_->setText("Stop Logging (R)");
    }
    if (log_status_)
    {
      log_status_->setText(QString("log: %1").arg(file_path));
    }

    if (!log_timer_)
    {
      log_timer_ = new QTimer(this);
      log_timer_->setInterval(100);
      connect(log_timer_, &QTimer::timeout, this, &ImuJointWidget::logSample);
    }
    log_timer_->start();
    return;
  }

  logging_enabled_ = false;
  if (log_button_)
  {
    log_button_->setText("Start Logging (R)");
  }
  if (log_status_)
  {
    log_status_->setText("log: idle");
  }
  if (log_timer_)
  {
    log_timer_->stop();
  }
  if (log_file_.isOpen())
  {
    log_file_.close();
  }
}

void ImuJointWidget::logSample()
{
  if (!logging_enabled_ || !has_state_ || !log_file_.isOpen())
  {
    return;
  }

  const TitaState &s = latest_state_;
  const double t = timer_.elapsed() / 1000.0;

  log_stream_ << QString::number(t, 'f', 6) << ';'
              << QString::number(s.roll, 'f', 6) << ';'
              << QString::number(s.pitch, 'f', 6) << ';'
              << QString::number(s.yaw, 'f', 6) << ';'
              << QString::number(s.Left.Pos.joint1, 'f', 6) << ';'
              << QString::number(s.Left.Pos.joint2, 'f', 6) << ';'
              << QString::number(s.Left.Pos.joint3, 'f', 6) << ';'
              << QString::number(s.Left.Pos.Wheel, 'f', 6) << ';'
              << QString::number(s.Left.Vel.joint1, 'f', 6) << ';'
              << QString::number(s.Left.Vel.joint2, 'f', 6) << ';'
              << QString::number(s.Left.Vel.joint3, 'f', 6) << ';'
              << QString::number(s.Left.Vel.Wheel, 'f', 6) << ';'
              << QString::number(s.Left.Effort.joint1, 'f', 6) << ';'
              << QString::number(s.Left.Effort.joint2, 'f', 6) << ';'
              << QString::number(s.Left.Effort.joint3, 'f', 6) << ';'
              << QString::number(s.Left.Effort.Wheel, 'f', 6) << ';'
              << QString::number(s.Right.Pos.joint1, 'f', 6) << ';'
              << QString::number(s.Right.Pos.joint2, 'f', 6) << ';'
              << QString::number(s.Right.Pos.joint3, 'f', 6) << ';'
              << QString::number(s.Right.Pos.Wheel, 'f', 6) << ';'
              << QString::number(s.Right.Vel.joint1, 'f', 6) << ';'
              << QString::number(s.Right.Vel.joint2, 'f', 6) << ';'
              << QString::number(s.Right.Vel.joint3, 'f', 6) << ';'
              << QString::number(s.Right.Vel.Wheel, 'f', 6) << ';'
              << QString::number(s.Right.Effort.joint1, 'f', 6) << ';'
              << QString::number(s.Right.Effort.joint2, 'f', 6) << ';'
              << QString::number(s.Right.Effort.joint3, 'f', 6) << ';'
              << QString::number(s.Right.Effort.Wheel, 'f', 6) << '\n';
  log_stream_.flush();
}
