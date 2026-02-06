#include "tita_ui/ros/ControlPublisher.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>

#include "tita_ui/TitaState.hpp"

ControlPublisher::ControlPublisher(const rclcpp::Node::SharedPtr &node, const std::string &topic)
  : node_(node)
{
  publisher_ = node_->create_publisher<std_msgs::msg::Float64MultiArray>(topic, rclcpp::QoS(10));
  timer_ = node_->create_wall_timer(std::chrono::milliseconds(2), [this]() { onTimer(); });
}

namespace
{
double pdCalculate(double kp, double kd, double th_ref, double dth_ref, double th, double dth)
{
  const double th_error = th_ref - th;
  const double dth_error = dth_ref - dth;
  return kp * th_error + kd * dth_error;
}

double taskPosition(double t, double T, double start, double end)
{
  if (T <= 0.0)
  {
    return end;
  }
  const double tc = std::max(0.0, std::min(t, T));
  constexpr double kPi = 3.14159265358979323846;
  const double delta = end - start;
  return start + delta * 0.5 * (1.0 - std::cos(kPi * tc / T));
}

double taskVelocity(double t, double T, double start, double end)
{
  if (T <= 0.0)
  {
    return 0.0;
  }
  const double tc = std::max(0.0, std::min(t, T));
  constexpr double kPi = 3.14159265358979323846;
  const double delta = end - start;
  return delta * 0.5 * (kPi / T) * std::sin(kPi * tc / T);
}
}  // namespace

void ControlPublisher::onTimer()
{
  std_msgs::msg::Float64MultiArray msg;
  msg.data.resize(3);

  static std::uint64_t last_seq = 0;
  static bool active = false;
  static double start_time = 0.0;
  static double start_pos[3] = {};
  static double start_vel[3] = {};
  static double target[3] = {};
  static double duration = 0.0;
  static double kp[3] = {};
  static double kd[3] = {};
  static std::uint64_t last_stop_seq = 0;

  const double now = node_->get_clock()->now().seconds();
  {
    std::lock_guard<std::mutex> lock(tita_state_mutex);
    if (tita_state.Control.stop_seq != last_stop_seq)
    {
      last_stop_seq = tita_state.Control.stop_seq;
      active = false;
      tita_state.Right.Command.Effort.joint1 = 0.0;
      tita_state.Right.Command.Effort.joint2 = 0.0;
      tita_state.Right.Command.Effort.joint3 = 0.0;
    }

    if (tita_state.Control.seq != last_seq)
    {
      last_seq = tita_state.Control.seq;
      active = true;
      start_time = now;
      start_pos[0] = tita_state.Right.Real.Pos.joint1;
      start_pos[1] = tita_state.Right.Real.Pos.joint2;
      start_pos[2] = tita_state.Right.Real.Pos.joint3;
      start_vel[0] = tita_state.Right.Real.Vel.joint1;
      start_vel[1] = tita_state.Right.Real.Vel.joint2;
      start_vel[2] = tita_state.Right.Real.Vel.joint3;
      target[0] = tita_state.Control.target_j1;
      target[1] = tita_state.Control.target_j2;
      target[2] = tita_state.Control.target_j3;
      duration = tita_state.Control.duration_sec;
      kp[0] = tita_state.Control.kp_j1;
      kp[1] = tita_state.Control.kp_j2;
      kp[2] = tita_state.Control.kp_j3;
      kd[0] = tita_state.Control.kd_j1;
      kd[1] = tita_state.Control.kd_j2;
      kd[2] = tita_state.Control.kd_j3;
    }

    if (active)
    {
      const double t = now - start_time;
      for (int i = 0; i < 3; ++i)
      {
        const double th_ref = taskPosition(t, duration, start_pos[i], target[i]);
        const double dth_ref = taskVelocity(t, duration, start_pos[i], target[i]);
        if (i == 0)
        {
          tita_state.Right.Command.Pos.joint1 = th_ref;
          tita_state.Right.Command.Vel.joint1 = dth_ref;
        }
        if (i == 1)
        {
          tita_state.Right.Command.Pos.joint2 = th_ref;
          tita_state.Right.Command.Vel.joint2 = dth_ref;
        }
        if (i == 2)
        {
          tita_state.Right.Command.Pos.joint3 = th_ref;
          tita_state.Right.Command.Vel.joint3 = dth_ref;
        }
        const double th = (i == 0) ? tita_state.Right.Real.Pos.joint1
                                   : (i == 1) ? tita_state.Right.Real.Pos.joint2
                                              : tita_state.Right.Real.Pos.joint3;
        const double dth = (i == 0) ? tita_state.Right.Real.Vel.joint1
                                    : (i == 1) ? tita_state.Right.Real.Vel.joint2
                                               : tita_state.Right.Real.Vel.joint3;
        const double effort = pdCalculate(kp[i], kd[i], th_ref, dth_ref, th, dth);
        if (i == 0) tita_state.Right.Command.Effort.joint1 = effort;
        if (i == 1) tita_state.Right.Command.Effort.joint2 = effort;
        if (i == 2) tita_state.Right.Command.Effort.joint3 = effort;
      }
      if (t > duration)
      {
        active = false;
      }
    }

    msg.data[0] = tita_state.Right.Command.Effort.joint1;
    msg.data[1] = tita_state.Right.Command.Effort.joint2;
    msg.data[2] = tita_state.Right.Command.Effort.joint3;
  }
  publisher_->publish(msg);
}
