#pragma once

#include <deque>

#include <QWidget>
#include <QPainterPath>

class SignalPlotWidget : public QWidget
{
  Q_OBJECT

public:
  explicit SignalPlotWidget(QWidget *parent = nullptr);

  void appendSample(int series, double time_sec, double value);
  void setTimeWindow(double seconds);
  void setSeriesColor(int series, const QColor &color);
  void clearSeries(int series);

protected:
  void paintEvent(QPaintEvent *event) override;

private:
  struct Sample
  {
    double t;
    double v;
  };

  std::deque<Sample> samples_[2];
  QColor series_colors_[2];
  double time_window_sec_{10.0};
};
