#include <thread>

#include <QApplication>

#include <rclcpp/rclcpp.hpp>

#include "tita_ui/ros/ImuJointSubscriber.hpp"
#include "tita_ui/ui/ImuJointWidget.hpp"
#include "tita_ui/TitaState.hpp"

Q_DECLARE_METATYPE(TitaState)

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  QApplication app(argc, argv);

  qRegisterMetaType<TitaState>("TitaState");

  auto node = std::make_shared<rclcpp::Node>("tita_ui");
  node->declare_parameter<std::string>("imu_topic", "/imu_sensor_broadcaster/imu");
  node->declare_parameter<std::string>("joint_states_topic", "/joint_states");

  const std::string imu_topic = node->get_parameter("imu_topic").as_string();
  const std::string joint_topic = node->get_parameter("joint_states_topic").as_string();

  ImuJointWidget widget;
  widget.setWindowTitle("Tita IMU & Joint States");
  widget.resize(900, 700);

  ImuJointSubscriber subscriber(node, imu_topic, joint_topic);
  QObject::connect(&subscriber, &ImuJointSubscriber::titaStateUpdated,
                   &widget, &ImuJointWidget::onTitaStateUpdated, Qt::QueuedConnection);

  widget.showMaximized();

  rclcpp::executors::SingleThreadedExecutor executor;
  executor.add_node(node);
  std::thread ros_thread([&executor]() { executor.spin(); });

  const int result = app.exec();

  executor.cancel();
  rclcpp::shutdown();
  if (ros_thread.joinable())
  {
    ros_thread.join();
  }

  return result;
}
