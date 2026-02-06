#include "tita_ui/ui/SignalPlotWidget.hpp"

#include <algorithm>

#include <QPainter>

SignalPlotWidget::SignalPlotWidget(QWidget *parent)
  : QWidget(parent)
{
  setMinimumHeight(220);
  setMinimumWidth(220);
  series_colors_[0] = QColor(60, 120, 200);
  series_colors_[1] = QColor(220, 90, 80);
}

void SignalPlotWidget::setTimeWindow(double seconds)
{
  time_window_sec_ = seconds;
}

void SignalPlotWidget::setSeriesColor(int series, const QColor &color)
{
  if (series < 0 || series > 1)
  {
    return;
  }
  series_colors_[series] = color;
}

void SignalPlotWidget::clearSeries(int series)
{
  if (series < 0 || series > 1)
  {
    return;
  }
  samples_[series].clear();
  update();
}

void SignalPlotWidget::appendSample(int series, double time_sec, double value)
{
  if (series < 0 || series > 1)
  {
    return;
  }

  samples_[series].push_back({time_sec, value});

  auto &series_samples = samples_[series];
  while (!series_samples.empty() && (time_sec - series_samples.front().t) > time_window_sec_)
  {
    series_samples.pop_front();
  }

  update();
}

void SignalPlotWidget::paintEvent(QPaintEvent *event)
{
  Q_UNUSED(event);

  QPainter painter(this);
  painter.setRenderHint(QPainter::Antialiasing, true);

  const QRectF bounds = rect().adjusted(6, 6, -6, -6);
  painter.fillRect(bounds, QColor(250, 250, 250));
  painter.setPen(QColor(210, 210, 210));
  painter.drawRect(bounds);

  if (samples_[0].size() < 2 && samples_[1].size() < 2)
  {
    painter.setPen(QColor(160, 160, 160));
    painter.drawText(bounds, Qt::AlignCenter, "No data");
    return;
  }

  double now = 0.0;
  if (!samples_[0].empty())
  {
    now = samples_[0].back().t;
  }
  if (!samples_[1].empty())
  {
    now = std::max(now, samples_[1].back().t);
  }
  const double t_min = now - time_window_sec_;
  const double t_max = now;

  bool initialized = false;
  double v_min = 0.0;
  double v_max = 0.0;
  for (const auto &series : samples_)
  {
    for (const auto &s : series)
    {
      if (!initialized)
      {
        v_min = s.v;
        v_max = s.v;
        initialized = true;
      }
      else
      {
        v_min = std::min(v_min, s.v);
        v_max = std::max(v_max, s.v);
      }
    }
  }

  if (v_max - v_min < 1e-6)
  {
    v_max = v_min + 1.0;
  }

  const double v_pad = (v_max - v_min) * 0.1;
  v_min -= v_pad;
  v_max += v_pad;

  auto map_x = [&](double t)
  {
    return bounds.left() + (t - t_min) / (t_max - t_min) * bounds.width();
  };
  auto map_y = [&](double v)
  {
    return bounds.bottom() - (v - v_min) / (v_max - v_min) * bounds.height();
  };

  for (int series = 0; series < 2; ++series)
  {
    const auto &data = samples_[series];
    if (data.size() < 2)
    {
      continue;
    }
    QPainterPath path;
    path.moveTo(map_x(data.front().t), map_y(data.front().v));
    for (size_t i = 1; i < data.size(); ++i)
    {
      path.lineTo(map_x(data[i].t), map_y(data[i].v));
    }
    painter.setPen(QPen(series_colors_[series], 1.8));
    painter.drawPath(path);
  }

  const double current = !samples_[0].empty() ? samples_[0].back().v : samples_[1].back().v;
  painter.setPen(QColor(80, 80, 80));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignTop | Qt::AlignLeft,
                   QString("min %1").arg(QString::number(v_min, 'f', 3)));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignTop | Qt::AlignRight,
                   QString("max %1").arg(QString::number(v_max, 'f', 3)));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignBottom | Qt::AlignRight,
                   QString("now %1").arg(QString::number(current, 'f', 3)));
}
