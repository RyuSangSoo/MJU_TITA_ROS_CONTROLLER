#pragma once

#include <functional>
#include <vector>

#include <QDateTime>
#include <QDir>
#include <QElapsedTimer>
#include <QFile>
#include <QPushButton>
#include <QStandardPaths>
#include <QTimer>
#include <QString>
#include <QKeyEvent>
#include <QFileInfo>
#include <QDoubleSpinBox>
#include <QTableWidget>
#include <QWidget>
#include <QTextStream>

#include "tita_ui/TitaState.hpp"

class QLabel;
class SignalPlotWidget;

class ImuJointWidget : public QWidget
{
  Q_OBJECT

public:
  explicit ImuJointWidget(QWidget *parent = nullptr);

public slots:
  void onTitaStateUpdated(const TitaState &state);

protected:
  void keyPressEvent(QKeyEvent *event) override;

private:
  void refreshUi();
  void toggleLogging();
  void logSample();

  QLabel *tita_roll_{nullptr};
  QLabel *tita_pitch_{nullptr};
  QLabel *tita_yaw_{nullptr};
  QPushButton *log_button_{nullptr};
  QLabel *log_status_{nullptr};
  QDoubleSpinBox *target_j1_{nullptr};
  QDoubleSpinBox *target_j2_{nullptr};
  QDoubleSpinBox *target_j3_{nullptr};
  QDoubleSpinBox *duration_sec_{nullptr};
  QDoubleSpinBox *kp_j1_{nullptr};
  QDoubleSpinBox *kp_j2_{nullptr};
  QDoubleSpinBox *kp_j3_{nullptr};
  QDoubleSpinBox *kd_j1_{nullptr};
  QDoubleSpinBox *kd_j2_{nullptr};
  QDoubleSpinBox *kd_j3_{nullptr};
  QPushButton *send_button_{nullptr};
  QPushButton *stop_button_{nullptr};

  QTableWidget *tita_left_table_{nullptr};
  QTableWidget *tita_right_table_{nullptr};

  static QString formatDouble(double value);

  struct PlotBinding
  {
    QString name;
    SignalPlotWidget *plot{nullptr};
    std::function<double(const TitaState &)> getter_primary;
    std::function<double(const TitaState &)> getter_secondary;
  };

  QElapsedTimer timer_;
  std::vector<PlotBinding> plots_;

  QTimer *refresh_timer_{nullptr};
  TitaState latest_state_{};
  bool has_state_{false};

  QTimer *log_timer_{nullptr};
  QFile log_file_;
  QTextStream log_stream_;
  bool logging_enabled_{false};
};
