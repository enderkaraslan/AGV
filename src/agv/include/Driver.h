#pragma once

#include "rclcpp/rclcpp.hpp"
#include "nav_msgs/msg/odometry.hpp"            //< Odometry
#include "visualization_msgs/msg/marker.hpp"    //< Marker
#include "nav_msgs/msg/occupancy_grid.hpp"      //< Occupancy Grid
#include "geometry_msgs/msg/pose_stamped.hpp"   //< Pose Stamped for navigation 
#include "nav_msgs/msg/path.hpp"


#include <mutex>
#include <opencv2/opencv.hpp>

#include "Constants.h"
#include "MotorNode.h"
#include "Timer.h"
#include "Route.h"
#include "Algorithm.h"


#include "my_robot_interfaces/msg/image_info.hpp"
#include "my_robot_interfaces/msg/lidar_alert.hpp"

namespace AGV
{
struct OccupancyGridData
{
    float resolution_{0.0};
    double originX_{0.0};
    double originY_{0.0};
    uint32_t width_{0};
    uint32_t height_{0};
    std::vector<int8_t> data_;
};

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

    // Subscribe to occupancy grid
    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occupancy_grid_subscriber_;
    rclcpp::CallbackGroup::SharedPtr occupancy_grid_callback_group_;

    // SUbscribe to path topic
    rclcpp::Subscription<nav_msgs::msg::Path>::SharedPtr path_subscriber_;
    rclcpp::CallbackGroup::SharedPtr path_callback_group_;

    // Publish marker for visualization
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr marker_publisher_;

    // PUblish 2D goal pose for navigation
    rclcpp::Publisher<geometry_msgs::msg::PoseStamped>::SharedPtr goal_pose_publisher_;

    // Callback function for image info 
    void imageInfoCallback(const my_robot_interfaces::msg::ImageInfo::SharedPtr& msg);

    // Callback function for lidar alert
    void lidarAlertCallback(const my_robot_interfaces::msg::LidarAlert::SharedPtr& msg);

    // Callback function for odometry
    void odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg);

    // Callback function for occupancy grid
    void occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg);

    // Callback function for path
    void pathCallback(const nav_msgs::msg::Path::SharedPtr& msg);

    MotorNode motor_;
    Timer my_timer_;
    Timer obstacle_detected_timer_;
    Timer nav2_timer_;
    Route my_route_;

    // Senkronizasyon ve Erişim Kontrolü
    std::mutex my_mutex_;
    std::atomic<bool> is_obstacle_detected{false};
    std::atomic<bool> turning_obstacle_detected_{false};

    // Hareket ve Yönelim ile İlgili Üyeler
    std::atomic<double> yaw_z_{0.0};
    double initial_yaw_{0.0};
    double distance_travelled_{0.0};
    bool is_turn_in_progress_{true};
    int8_t direction_; 
    float x_{0.0};
    float y_{0.0};
    bool obstacle_detected_flag_{true};

    double q_x = 0.0;
    double q_y = 0.0;
    double q_z = 0.0;
    double w = 0.0;
    // NAV2 İÇİN FLAG
    bool is_nav2_enabled_{false};

    // Yardımcı Fonksiyonlar
    double eulerFromQuaternion(double x, double y, double z, double w);
    std::vector<std::vector<int8_t>> costmap(const std::vector<int8_t>& data, uint32_t width, uint32_t height, float resolution, uint8_t expansion_size);

    OccupancyGridData occupancy_grid_data_;


};  // Class Driver

}   // namespace AGV