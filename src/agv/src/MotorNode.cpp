#include "MotorNode.h"

namespace AGV
{

MotorNode::MotorNode() : Node("motor_node")
{
    speed_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
}

MotorNode::~MotorNode() 
{
    RCLCPP_DEBUG(this->get_logger(), "MotorNode has been stopped.");
}

void MotorNode::publishSpeed() 
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x  = linear_speed_;
    twist.angular.z = angular_speed_;
    
    if (speed_publisher_) 
    {
        speed_publisher_->publish(twist);
    } 
    else 
    {

        RCLCPP_WARN(this->get_logger(), "Publisher not initialized!");
    }
}

}   // namespace AGV
