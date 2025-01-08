#pragma once

#include "rclcpp/rclcpp.hpp"

#include <mutex>
#include "Constants.h"
#include "MotorNode.h"
#include "Timer.h"
#include "Route.h"


#include "nav_msgs/msg/odometry.hpp"
#include "my_robot_interfaces/msg/image_info.hpp"
#include "my_robot_interfaces/msg/lidar_alert.hpp"

namespace AGV
{

class Driver : public rclcpp::Node
{
public:
    // Constructor and destructor for Driver class
    Driver();
    ~Driver();

private:
    // Subscribe to image_info
    rclcpp::Subscription<my_robot_interfaces::msg::ImageInfo>::SharedPtr image_info_subscriber_;
    rclcpp::CallbackGroup::SharedPtr image_info_callback_group_;

    // Subscribe to lidar_alert
    rclcpp::Subscription<my_robot_interfaces::msg::LidarAlert>::SharedPtr lidar_alert_subscriber_;    
    rclcpp::CallbackGroup::SharedPtr lidar_alert_callback_group_;

    // Subscribe to odometry
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odometry_subscriber_;
    rclcpp::CallbackGroup::SharedPtr odometry_callback_group_;


    // Callback function for image info 
    void imageInfoCallback(const my_robot_interfaces::msg::ImageInfo::SharedPtr& msg);

    // Callback function for lidar alert
    void lidarAlertCallback(const my_robot_interfaces::msg::LidarAlert::SharedPtr& msg);

    // Callback function for odometry
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);

    MotorNode motor_;
    Timer my_timer_;
    Route my_route_;

    std::mutex my_mutex_;

    std::atomic<bool> is_obstacle_detected{false};
    std::atomic<double> yaw_z_{0.0};
    std::atomic<bool> turning_obstacle_detected_{false};
    double initial_yaw_{0.0};
    
    double distance_travelled_{0.0};

    bool is_turn_in_progress_{true}; 

    int8_t direction_;

    double eulerFromQuaternion(double x, double y, double z, double w);

};  // Class Driver

}   // namespace AGV