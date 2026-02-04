#include "leg_controller.hpp"
#include "state.hpp"

#include <algorithm>
#include <cmath>

Tita tita_state{};

leg_controller::leg_controller() : Node("leg_controller")
{
    effort_pub = this->create_publisher<std_msgs::msg::Float64MultiArray>("/tita_hw/effort_controller/commands", 1);

    jointdata = this->create_subscription<sensor_msgs::msg::JointState>(
        "/tita_hw/joint_states", 1,
        std::bind(&leg_controller::JointStateCallBack, this, std::placeholders::_1));
}

void leg_controller::JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg)
{
    std::unordered_map<std::string, double> pos_map;
    std::unordered_map<std::string, double> vel_map;

    for (size_t i = 0; i < JointMsg->name.size(); ++i) {
        if (i < JointMsg->position.size()) {
            pos_map[JointMsg->name[i]] = JointMsg->position[i];
        }
        if (i < JointMsg->velocity.size()) {
            vel_map[JointMsg->name[i]] = JointMsg->velocity[i];
        }
    }

    auto assign_if_found = [this](const std::unordered_map<std::string, double> &map,
                                  const std::string &joint_name,
                                  double &target,
                                  const char *label) {
        const auto it = map.find(joint_name);
        if (it != map.end()) {
            target = it->second;
            return;
        }
        RCLCPP_WARN(this->get_logger(), "Missing %s data for joint: %s", label, joint_name.c_str());
    };

    assign_if_found(pos_map, "joint_right_leg_1", tita_state.Act.right.Pos.joint1, "position");
    assign_if_found(pos_map, "joint_right_leg_2", tita_state.Act.right.Pos.joint2, "position");
    assign_if_found(pos_map, "joint_right_leg_3", tita_state.Act.right.Pos.joint3, "position");
    assign_if_found(pos_map, "joint_right_leg_4", tita_state.Act.right.Pos.joint4, "position");

    assign_if_found(pos_map, "joint_left_leg_1", tita_state.Act.left.Pos.joint1, "position");
    assign_if_found(pos_map, "joint_left_leg_2", tita_state.Act.left.Pos.joint2, "position");
    assign_if_found(pos_map, "joint_left_leg_3", tita_state.Act.left.Pos.joint3, "position");
    assign_if_found(pos_map, "joint_left_leg_4", tita_state.Act.left.Pos.joint4, "position");

    assign_if_found(vel_map, "joint_right_leg_1", tita_state.Act.right.Vel.joint1, "velocity");
    assign_if_found(vel_map, "joint_right_leg_2", tita_state.Act.right.Vel.joint2, "velocity");
    assign_if_found(vel_map, "joint_right_leg_3", tita_state.Act.right.Vel.joint3, "velocity");
    assign_if_found(vel_map, "joint_right_leg_4", tita_state.Act.right.Vel.joint4, "velocity");

    assign_if_found(vel_map, "joint_left_leg_1", tita_state.Act.left.Vel.joint1, "velocity");
    assign_if_found(vel_map, "joint_left_leg_2", tita_state.Act.left.Vel.joint2, "velocity");
    assign_if_found(vel_map, "joint_left_leg_3", tita_state.Act.left.Vel.joint3, "velocity");
    assign_if_found(vel_map, "joint_left_leg_4", tita_state.Act.left.Vel.joint4, "velocity");

    has_joint_state_.store(true, std::memory_order_relaxed);
}

bool leg_controller::HasJointState() const
{
    return has_joint_state_.load(std::memory_order_relaxed);
}

// PD Controller
pdcontroller::pdcontroller(double kp, double kd)
    : Kp(kp), Kd(kd), previous_error(0) {}

double pdcontroller::pdcalculate(double th_ref, double dth_ref, double th, double dth) {
    // P Calculate
    double th_error = th_ref - th;

    // D Calculate
    double dth_error = dth_ref - dth;

    double output = Kp*th_error + Kd*dth_error;

    return output;
}

double rss_controller::TaskPosition(double t, double T, double start, double end) {
    if (T <= 0.0) {
        return end;
    }
    const double tc = std::max(0.0, std::min(t, T));
    constexpr double kPi = 3.14159265358979323846;
    double delta = end - start;
    return start + delta * 0.5 * (1.0 - std::cos(kPi * tc / T));
}

double rss_controller::TaskVelocity(double t, double T, double start, double end) {
    if (T <= 0.0) {
        return 0.0;
    }
    const double tc = std::max(0.0, std::min(t, T));
    constexpr double kPi = 3.14159265358979323846;
    double delta = end - start;
    return delta * 0.5 * (kPi / T) * std::sin(kPi * tc / T);
}
