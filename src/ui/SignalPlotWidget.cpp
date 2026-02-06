#include "tita_ui/ui/SignalPlotWidget.hpp"

#include <algorithm>

#include <QPainter>

SignalPlotWidget::SignalPlotWidget(QWidget *parent)
  : QWidget(parent)
{
  setMinimumHeight(220);
  setMinimumWidth(220);
}

void SignalPlotWidget::setTimeWindow(double seconds)
{
  time_window_sec_ = seconds;
}

void SignalPlotWidget::appendSample(double time_sec, double value)
{
  samples_.push_back({time_sec, value});

  while (!samples_.empty() && (time_sec - samples_.front().t) > time_window_sec_)
  {
    samples_.pop_front();
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

  if (samples_.size() < 2)
  {
    painter.setPen(QColor(160, 160, 160));
    painter.drawText(bounds, Qt::AlignCenter, "No data");
    return;
  }

  const double now = samples_.back().t;
  const double t_min = now - time_window_sec_;
  const double t_max = now;

  double v_min = samples_.front().v;
  double v_max = samples_.front().v;
  for (const auto &s : samples_)
  {
    v_min = std::min(v_min, s.v);
    v_max = std::max(v_max, s.v);
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

  QPainterPath path;
  path.moveTo(map_x(samples_.front().t), map_y(samples_.front().v));
  for (size_t i = 1; i < samples_.size(); ++i)
  {
    path.lineTo(map_x(samples_[i].t), map_y(samples_[i].v));
  }

  painter.setPen(QPen(QColor(60, 120, 200), 1.8));
  painter.drawPath(path);

  const double current = samples_.back().v;
  painter.setPen(QColor(80, 80, 80));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignTop | Qt::AlignLeft,
                   QString("min %1").arg(QString::number(v_min, 'f', 3)));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignTop | Qt::AlignRight,
                   QString("max %1").arg(QString::number(v_max, 'f', 3)));
  painter.drawText(bounds.adjusted(4, 4, -4, -4), Qt::AlignBottom | Qt::AlignRight,
                   QString("now %1").arg(QString::number(current, 'f', 3)));
}
