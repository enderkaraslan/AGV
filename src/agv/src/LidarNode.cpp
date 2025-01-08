#include "LidarNode.h"

namespace AGV
{
    //Constructor function
    LidarNode::LidarNode() : Node("lidar_node")
    {
        //Lidar subsystem
        lidar_subscription_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            Constants::LidarTopic, Constants::QueueSize,
            [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                lidarCallback(msg);
            });

        //Lidar publisher
        lidar_info_publisher_ = this->create_publisher<my_robot_interfaces::msg::LidarAlert>(
            Constants::LidarAlertTopic, Constants::QueueSize);
    
        RCLCPP_DEBUG(this->get_logger(), "LidarNode has been started.");

    }

    //Destructor function
    LidarNode::~LidarNode()
    {
        RCLCPP_DEBUG(this->get_logger(), "LidarNode has been stopped.");
    }

    //Callback function for lidar data
    void LidarNode::lidarCallback(const sensor_msgs::msg::LaserScan::SharedPtr& msg)
    {
        // Find the minimum distance in the lidar scan
        float min_distance = Constants::MaximumDistance;

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            // Update minimum distance if current distance is smaller than the previous one
            if (msg->ranges[i] < min_distance)
            {
                min_distance = msg->ranges[i];
            }

            // If an obstacle is detected within the detection distance, publish the alert message and return
            if (msg->ranges[i] < Constants::DetectionDistance)
            {
                RCLCPP_DEBUG(this->get_logger(), "Obstacle Detected: Distance = %.2f", msg->ranges[i]);

                auto message                = my_robot_interfaces::msg::LidarAlert();

                message.is_obstacle_detect  = true;
                message.distance            = msg->ranges[i];
                
                lidar_info_publisher_->publish(message);
                return; 
            }
        }

        // No obstacle detected, publish the alert message with the minimum distance
        RCLCPP_DEBUG(this->get_logger(), "No obstacle detected");

        auto message                = my_robot_interfaces::msg::LidarAlert();
        
        message.is_obstacle_detect  = false;
        message.distance            = min_distance;

        lidar_info_publisher_->publish(message);
    }
}