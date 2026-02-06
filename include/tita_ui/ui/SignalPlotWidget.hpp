#pragma once

#include <deque>

#include <QWidget>

class SignalPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SignalPlotWidget(QWidget *parent = nullptr);

  void appendSample(double time_sec, double value);
  void setTimeWindow(double seconds);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  struct Sample
  {
    double t;
    double v;
  };

  std::deque<Sample> samples_;
  double time_window_sec_{10.0};
};
