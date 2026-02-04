#ifndef LEG_CONTROLLER_HPP
#define LEG_CONTROLLER_HPP

#include "header.hpp"

class leg_controller : public rclcpp::Node
{
private:
    std::atomic<bool> has_joint_state_{false};

public:
    leg_controller();
    ~leg_controller() override = default;

    rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr effort_pub;

    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr jointdata;

    void JointStateCallBack(const sensor_msgs::msg::JointState::SharedPtr JointMsg);
    bool HasJointState() const;

};

// PD 제어기
class pdcontroller
{
private:
    double Kp;
    double Kd;
    double previous_error;
    double output_Gen;
public:
    pdcontroller(double kp, double kd);
    double pdcalculate(double th_ref, double dth_ref, double th, double dth);
};

class rss_controller
{
private:
    /* data */
public:

    double TaskPosition(double t, double T, double start, double end);
    double TaskVelocity(double t, double T, double start, double end);
};


#endif // LEG_CONTROLLER_HPP
