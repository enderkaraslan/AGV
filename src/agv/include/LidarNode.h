#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include "std_msgs/msg/string.hpp"

#include <limits>

#include "my_robot_interfaces/msg/lidar_alert.hpp"
#include "Constants.h"

namespace AGV
{
class LidarNode : public rclcpp::Node
{
public:
    LidarNode();
    ~LidarNode();

private:
    // Lidar Message 
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr lidar_subscription_;

    // Publishing Messages
    rclcpp::Publisher<my_robot_interfaces::msg::LidarAlert>::SharedPtr lidar_info_publisher_;    
    
    // Subscribe to lidar messages
    void lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg);

};   // class LidarNode

}   // namespace AGV
