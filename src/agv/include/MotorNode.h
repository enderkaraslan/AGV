#pragma once

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"

namespace AGV
{
// Motor class for controlling the robot's motors
class MotorNode : public rclcpp::Node
{

public:
    // Constructor and destructor for Motor class
    MotorNode();
    ~MotorNode();

    // Speed publish
    void publishSpeed();

    // Speed getters and setters
    double getLinearSpeed() const { return linear_speed_; }
    double getAngularSpeed() const { return angular_speed_; }

    void setLinearSpeed(double linear_speed) { linear_speed_ = linear_speed; }
    void setAngularSpeed(double angular_speed) { angular_speed_ = angular_speed; }

private:
    // Speed publisher implementation
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;

    // Speed variables
    double linear_speed_;
    double angular_speed_;

};  // Class MotorNode

}   // namespace AGV
