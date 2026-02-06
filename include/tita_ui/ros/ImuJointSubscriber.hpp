#pragma once

#include <QObject>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/imu.hpp>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tita_ui/TitaState.hpp"

class ImuJointSubscriber : public QObject
{
  Q_OBJECT

public:
  ImuJointSubscriber(
    const rclcpp::Node::SharedPtr &node,
    const std::string &imu_topic,
    const std::string &joint_topic);

signals:
  void titaStateUpdated(const TitaState &state);

private:
  rclcpp::Subscription<sensor_msgs::msg::Imu>::SharedPtr imu_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_sub_;
};
