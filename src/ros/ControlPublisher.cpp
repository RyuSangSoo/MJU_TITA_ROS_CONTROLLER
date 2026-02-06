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
  msg.data.resize(6);

  static std::uint64_t last_seq_left = 0;
  static std::uint64_t last_seq_right = 0;
  static std::uint64_t last_stop_seq_left = 0;
  static std::uint64_t last_stop_seq_right = 0;
  static bool active_left = false;
  static bool active_right = false;
  static double start_time_left = 0.0;
  static double start_time_right = 0.0;
  static double start_pos_left[3] = {};
  static double start_pos_right[3] = {};
  static double target_left[3] = {};
  static double target_right[3] = {};
  static double duration_left = 0.0;
  static double duration_right = 0.0;
  static double kp_left[3] = {};
  static double kd_left[3] = {};
  static double kp_right[3] = {};
  static double kd_right[3] = {};

  const double now = node_->get_clock()->now().seconds();
  {
    std::lock_guard<std::mutex> lock(tita_state_mutex);
    if (tita_state.ControlLeft.stop_seq != last_stop_seq_left)
    {
      last_stop_seq_left = tita_state.ControlLeft.stop_seq;
      active_left = false;
      tita_state.Left.Command.Effort.joint1 = 0.0;
      tita_state.Left.Command.Effort.joint2 = 0.0;
      tita_state.Left.Command.Effort.joint3 = 0.0;
    }

    if (tita_state.ControlRight.stop_seq != last_stop_seq_right)
    {
      last_stop_seq_right = tita_state.ControlRight.stop_seq;
      active_right = false;
      tita_state.Right.Command.Effort.joint1 = 0.0;
      tita_state.Right.Command.Effort.joint2 = 0.0;
      tita_state.Right.Command.Effort.joint3 = 0.0;
    }

    if (tita_state.ControlLeft.seq != last_seq_left)
    {
      last_seq_left = tita_state.ControlLeft.seq;
      active_left = true;
      start_time_left = now;
      start_pos_left[0] = tita_state.Left.Real.Pos.joint1;
      start_pos_left[1] = tita_state.Left.Real.Pos.joint2;
      start_pos_left[2] = tita_state.Left.Real.Pos.joint3;
      target_left[0] = tita_state.ControlLeft.target_j1;
      target_left[1] = tita_state.ControlLeft.target_j2;
      target_left[2] = tita_state.ControlLeft.target_j3;
      duration_left = tita_state.ControlLeft.duration_sec;
      kp_left[0] = tita_state.ControlLeft.kp_j1;
      kp_left[1] = tita_state.ControlLeft.kp_j2;
      kp_left[2] = tita_state.ControlLeft.kp_j3;
      kd_left[0] = tita_state.ControlLeft.kd_j1;
      kd_left[1] = tita_state.ControlLeft.kd_j2;
      kd_left[2] = tita_state.ControlLeft.kd_j3;
    }

    if (tita_state.ControlRight.seq != last_seq_right)
    {
      last_seq_right = tita_state.ControlRight.seq;
      active_right = true;
      start_time_right = now;
      start_pos_right[0] = tita_state.Right.Real.Pos.joint1;
      start_pos_right[1] = tita_state.Right.Real.Pos.joint2;
      start_pos_right[2] = tita_state.Right.Real.Pos.joint3;
      target_right[0] = tita_state.ControlRight.target_j1;
      target_right[1] = tita_state.ControlRight.target_j2;
      target_right[2] = tita_state.ControlRight.target_j3;
      duration_right = tita_state.ControlRight.duration_sec;
      kp_right[0] = tita_state.ControlRight.kp_j1;
      kp_right[1] = tita_state.ControlRight.kp_j2;
      kp_right[2] = tita_state.ControlRight.kp_j3;
      kd_right[0] = tita_state.ControlRight.kd_j1;
      kd_right[1] = tita_state.ControlRight.kd_j2;
      kd_right[2] = tita_state.ControlRight.kd_j3;
    }

    if (active_left)
    {
      const double t = now - start_time_left;
      for (int i = 0; i < 3; ++i)
      {
        const double th_ref = taskPosition(t, duration_left, start_pos_left[i], target_left[i]);
        const double dth_ref = taskVelocity(t, duration_left, start_pos_left[i], target_left[i]);
        if (i == 0)
        {
          tita_state.Left.Command.Pos.joint1 = th_ref;
          tita_state.Left.Command.Vel.joint1 = dth_ref;
        }
        if (i == 1)
        {
          tita_state.Left.Command.Pos.joint2 = th_ref;
          tita_state.Left.Command.Vel.joint2 = dth_ref;
        }
        if (i == 2)
        {
          tita_state.Left.Command.Pos.joint3 = th_ref;
          tita_state.Left.Command.Vel.joint3 = dth_ref;
        }
        const double th = (i == 0) ? tita_state.Left.Real.Pos.joint1
                                   : (i == 1) ? tita_state.Left.Real.Pos.joint2
                                              : tita_state.Left.Real.Pos.joint3;
        const double dth = (i == 0) ? tita_state.Left.Real.Vel.joint1
                                    : (i == 1) ? tita_state.Left.Real.Vel.joint2
                                               : tita_state.Left.Real.Vel.joint3;
        const double effort = pdCalculate(kp_left[i], kd_left[i], th_ref, dth_ref, th, dth);
        if (i == 0) tita_state.Left.Command.Effort.joint1 = effort;
        if (i == 1) tita_state.Left.Command.Effort.joint2 = effort;
        if (i == 2) tita_state.Left.Command.Effort.joint3 = effort;
      }
      if (t > duration_left)
      {
        active_left = false;
      }
    }

    if (active_right)
    {
      const double t = now - start_time_right;
      for (int i = 0; i < 3; ++i)
      {
        const double th_ref = taskPosition(t, duration_right, start_pos_right[i], target_right[i]);
        const double dth_ref = taskVelocity(t, duration_right, start_pos_right[i], target_right[i]);
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
        const double effort = pdCalculate(kp_right[i], kd_right[i], th_ref, dth_ref, th, dth);
        if (i == 0) tita_state.Right.Command.Effort.joint1 = effort;
        if (i == 1) tita_state.Right.Command.Effort.joint2 = effort;
        if (i == 2) tita_state.Right.Command.Effort.joint3 = effort;
      }
      if (t > duration_right)
      {
        active_right = false;
      }
    }

    msg.data[0] = tita_state.Left.Command.Effort.joint1;
    msg.data[1] = tita_state.Left.Command.Effort.joint2;
    msg.data[2] = tita_state.Left.Command.Effort.joint3;
    msg.data[3] = tita_state.Right.Command.Effort.joint1;
    msg.data[4] = tita_state.Right.Command.Effort.joint2;
    msg.data[5] = tita_state.Right.Command.Effort.joint3;
  }
  publisher_->publish(msg);
}
