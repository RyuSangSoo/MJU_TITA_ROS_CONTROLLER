#include "leg_controller.hpp"
#include "state.hpp"

#include <array>
#include <atomic>
#include <csignal>

namespace
{
constexpr double dt = 0.002;
constexpr double kTrajectoryTime = 2.0;
constexpr double kKp = 40.0;
constexpr double kKd = 1.5;
std::atomic<bool> g_stop_requested{false};

struct JointAxis
{
    double *ref_pos;
    double *ref_vel;
    double *act_pos;
    double *act_vel;
    double *effort;
    double start_pos;
    double target_pos;
};

std::array<JointAxis, 8> BuildAxes()
{
    // 원하는 목표 각도(rad): 아래 값만 바꾸면 됩니다.
    constexpr std::array<double, 8> target_positions = {
        0.35, 0.0, 0.0, 0.0,   // left  joint1~4
        -0.35, 0.0, 0.0, 0.0   // right joint1~4
    };

    std::array<JointAxis, 8> axes = {{
        {&tita_state.Ref.left.Pos.joint1, &tita_state.Ref.left.Vel.joint1, &tita_state.Act.left.Pos.joint1, &tita_state.Act.left.Vel.joint1, &tita_state.Act.left.Effort.joint1, 0.0, target_positions[0]},
        {&tita_state.Ref.left.Pos.joint2, &tita_state.Ref.left.Vel.joint2, &tita_state.Act.left.Pos.joint2, &tita_state.Act.left.Vel.joint2, &tita_state.Act.left.Effort.joint2, 0.0, target_positions[1]},
        {&tita_state.Ref.left.Pos.joint3, &tita_state.Ref.left.Vel.joint3, &tita_state.Act.left.Pos.joint3, &tita_state.Act.left.Vel.joint3, &tita_state.Act.left.Effort.joint3, 0.0, target_positions[2]},
        {&tita_state.Ref.left.Pos.joint4, &tita_state.Ref.left.Vel.joint4, &tita_state.Act.left.Pos.joint4, &tita_state.Act.left.Vel.joint4, &tita_state.Act.left.Effort.joint4, 0.0, target_positions[3]},
        {&tita_state.Ref.right.Pos.joint1, &tita_state.Ref.right.Vel.joint1, &tita_state.Act.right.Pos.joint1, &tita_state.Act.right.Vel.joint1, &tita_state.Act.right.Effort.joint1, 0.0, target_positions[4]},
        {&tita_state.Ref.right.Pos.joint2, &tita_state.Ref.right.Vel.joint2, &tita_state.Act.right.Pos.joint2, &tita_state.Act.right.Vel.joint2, &tita_state.Act.right.Effort.joint2, 0.0, target_positions[5]},
        {&tita_state.Ref.right.Pos.joint3, &tita_state.Ref.right.Vel.joint3, &tita_state.Act.right.Pos.joint3, &tita_state.Act.right.Vel.joint3, &tita_state.Act.right.Effort.joint3, 0.0, target_positions[6]},
        {&tita_state.Ref.right.Pos.joint4, &tita_state.Ref.right.Vel.joint4, &tita_state.Act.right.Pos.joint4, &tita_state.Act.right.Vel.joint4, &tita_state.Act.right.Effort.joint4, 0.0, target_positions[7]},
    }};

    return axes;
}

void CaptureStartPositions(std::array<JointAxis, 8> &axes)
{
    for (auto &axis : axes) {
        axis.start_pos = *axis.act_pos;
    }
}

void UpdateTrajectoryAndEffort(std::array<JointAxis, 8> &axes, double elapsed_time, rss_controller &traj, pdcontroller &pd)
{
    for (auto &axis : axes) {
        *axis.ref_pos = traj.TaskPosition(elapsed_time, kTrajectoryTime, axis.start_pos, axis.target_pos);
        *axis.ref_vel = traj.TaskVelocity(elapsed_time, kTrajectoryTime, axis.start_pos, axis.target_pos);
        *axis.effort = pd.pdcalculate(*axis.ref_pos, *axis.ref_vel, *axis.act_pos, *axis.act_vel);
    }
}

void FillEffortMessage(std_msgs::msg::Float64MultiArray &msg)
{
    msg.data = {tita_state.Act.left.Effort.joint1,
                tita_state.Act.left.Effort.joint2,
                tita_state.Act.left.Effort.joint3,
                tita_state.Act.left.Effort.joint4,
                tita_state.Act.right.Effort.joint1,
                tita_state.Act.right.Effort.joint2,
                tita_state.Act.right.Effort.joint3,
                tita_state.Act.right.Effort.joint4};
}

void SetAllEffortsZero()
{
    tita_state.Act.left.Effort.joint1 = 0.0;
    tita_state.Act.left.Effort.joint2 = 0.0;
    tita_state.Act.left.Effort.joint3 = 0.0;
    tita_state.Act.left.Effort.joint4 = 0.0;
    tita_state.Act.right.Effort.joint1 = 0.0;
    tita_state.Act.right.Effort.joint2 = 0.0;
    tita_state.Act.right.Effort.joint3 = 0.0;
    tita_state.Act.right.Effort.joint4 = 0.0;
}

void SignalHandler(int)
{
    g_stop_requested.store(true, std::memory_order_relaxed);
}
}

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    std::signal(SIGINT, SignalHandler);
    std::signal(SIGTERM, SignalHandler);

    auto leg_ctrl_node = std::make_shared<leg_controller>();
    pdcontroller joint_pd(kKp, kKd);
    rss_controller traj_gen;
    auto axes = BuildAxes();

    std_msgs::msg::Float64MultiArray effort_msg;
    rclcpp::Rate loop_rate(1.0 / dt);

    while (rclcpp::ok() && !g_stop_requested.load(std::memory_order_relaxed) && !leg_ctrl_node->HasJointState()) {
        rclcpp::spin_some(leg_ctrl_node);
        loop_rate.sleep();
    }

    if (!rclcpp::ok() || g_stop_requested.load(std::memory_order_relaxed)) {
        if (rclcpp::ok()) {
            SetAllEffortsZero();
            FillEffortMessage(effort_msg);
            leg_ctrl_node->effort_pub->publish(effort_msg);
        }
        rclcpp::shutdown();
        return 0;
    }

    CaptureStartPositions(axes);
    RCLCPP_INFO(leg_ctrl_node->get_logger(), "Trajectory start position captured. Moving to target.");

    double elapsed_time = 0.0;

    while (rclcpp::ok() && !g_stop_requested.load(std::memory_order_relaxed))
    {
        rclcpp::spin_some(leg_ctrl_node);
        UpdateTrajectoryAndEffort(axes, elapsed_time, traj_gen, joint_pd);
        FillEffortMessage(effort_msg);

        leg_ctrl_node->effort_pub->publish(effort_msg);
        if (elapsed_time < kTrajectoryTime) {
            elapsed_time += dt;
        }

        loop_rate.sleep();
    }

    // Safety stop: publish zero torques before shutdown.
    if (rclcpp::ok()) {
        SetAllEffortsZero();
        FillEffortMessage(effort_msg);
        for (int i = 0; i < 5; ++i) {
            leg_ctrl_node->effort_pub->publish(effort_msg);
            rclcpp::spin_some(leg_ctrl_node);
            loop_rate.sleep();
        }
    }
    
    rclcpp::shutdown();
    return 0;
}
