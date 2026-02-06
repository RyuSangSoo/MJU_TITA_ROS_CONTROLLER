#pragma once

#include <functional>
#include <vector>

#include <QElapsedTimer>
#include <QTimer>
#include <QString>
#include <QKeyEvent>
#include <QTableWidget>
#include <QWidget>

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

  QLabel *tita_roll_{nullptr};
  QLabel *tita_pitch_{nullptr};
  QLabel *tita_yaw_{nullptr};

  QTableWidget *tita_left_table_{nullptr};
  QTableWidget *tita_right_table_{nullptr};

  static QString formatDouble(double value);

  struct PlotBinding
  {
    QString name;
    SignalPlotWidget *plot{nullptr};
    std::function<double(const TitaState &)> getter;
  };

  QElapsedTimer timer_;
  std::vector<PlotBinding> plots_;

  QTimer *refresh_timer_{nullptr};
  TitaState latest_state_{};
  bool has_state_{false};
};
