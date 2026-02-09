#include "tita_ui/ros/ImuJointSubscriber.hpp"

#include <cmath>

ImuJointSubscriber::ImuJointSubscriber(
  const rclcpp::Node::SharedPtr &node,
  const std::string &imu_topic,
  const std::string &joint_topic)
{
  imu_sub_ = node->create_subscription<sensor_msgs::msg::Imu>(
    imu_topic, rclcpp::QoS(10),
    [this](const sensor_msgs::msg::Imu::SharedPtr msg)
    {
      const auto &q = msg->orientation;
      const double sinr_cosp = 2.0 * (q.w * q.x + q.y * q.z);
      const double cosr_cosp = 1.0 - 2.0 * (q.x * q.x + q.y * q.y);
      tita_state.roll = std::atan2(sinr_cosp, cosr_cosp);

      const double sinp = 2.0 * (q.w * q.y - q.z * q.x);
      constexpr double kHalfPi = 1.5707963267948966;
      if (std::abs(sinp) >= 1.0)
      {
        tita_state.pitch = std::copysign(kHalfPi, sinp);
      }
      else
      {
        tita_state.pitch = std::asin(sinp);
      }

      const double siny_cosp = 2.0 * (q.w * q.z + q.x * q.y);
      const double cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z);
      tita_state.yaw = std::atan2(siny_cosp, cosy_cosp);

      tita_state.angular_velocity_x = msg->angular_velocity.x;
      tita_state.angular_velocity_y = msg->angular_velocity.y;
      tita_state.angular_velocity_z = msg->angular_velocity.z;

      emit titaStateUpdated(tita_state);
    });

  joint_sub_ = node->create_subscription<sensor_msgs::msg::JointState>(
    joint_topic, rclcpp::QoS(10),
    [this](const sensor_msgs::msg::JointState::SharedPtr msg)
    {
      const auto &names = msg->name;
      for (size_t i = 0; i < names.size(); ++i)
      {
        auto write_joint = [&](bool left, int index)
        {
          auto &side = left ? tita_state.Left : tita_state.Right;
          if (i < msg->position.size())
          {
            if (index == 1) side.Pos.joint1 = msg->position[i];
            if (index == 2) side.Pos.joint2 = msg->position[i];
            if (index == 3) side.Pos.joint3 = msg->position[i];
            if (index == 4) side.Pos.Wheel = msg->position[i];
          }
          if (i < msg->velocity.size())
          {
            if (index == 1) side.Vel.joint1 = msg->velocity[i];
            if (index == 2) side.Vel.joint2 = msg->velocity[i];
            if (index == 3) side.Vel.joint3 = msg->velocity[i];
            if (index == 4) side.Vel.Wheel = msg->velocity[i];
          }
          if (i < msg->effort.size())
          {
            if (index == 1) side.Effort.joint1 = msg->effort[i];
            if (index == 2) side.Effort.joint2 = msg->effort[i];
            if (index == 3) side.Effort.joint3 = msg->effort[i];
            if (index == 4) side.Effort.Wheel = msg->effort[i];
          }
        };

        if (names[i] == "joint_left_leg_1") write_joint(true, 1);
        else if (names[i] == "joint_left_leg_2") write_joint(true, 2);
        else if (names[i] == "joint_left_leg_3") write_joint(true, 3);
        else if (names[i] == "joint_left_leg_4") write_joint(true, 4);
        else if (names[i] == "joint_right_leg_1") write_joint(false, 1);
        else if (names[i] == "joint_right_leg_2") write_joint(false, 2);
        else if (names[i] == "joint_right_leg_3") write_joint(false, 3);
        else if (names[i] == "joint_right_leg_4") write_joint(false, 4);
      }

      emit titaStateUpdated(tita_state);
    });
}
