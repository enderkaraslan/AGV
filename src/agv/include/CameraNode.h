#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/string.hpp"
#include "geometry_msgs/msg/twist.hpp"

#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#include "Constants.h"
#include "ImageProcessor.h"
#include "Route.h"
#include "Timer.h"


namespace AGV
{

class CameraNode : public rclcpp::Node
{

public:
    // Constructor and destructor for CameraNode class
    CameraNode();
    ~CameraNode();

private:
    // Camera Subscriber
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscriber_;
    rclcpp::CallbackGroup::SharedPtr camera_callback_group_;

    // State Subscriber
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr state_subscriber_;
    rclcpp::CallbackGroup::SharedPtr state_callback_group_;

    // State Publisher
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr state_publisher_;    

    // Speed Publisher
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr speed_publisher_;

    // Callback function for camera data
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg);

    // Callback function for state data
    void stateCallback(const std_msgs::msg::String::SharedPtr& msg);
    

    std::string state_{"LineFollow"};
    std::mutex state_mutex_;

    Route my_route_;
    Timer my_timer_;


    bool is_turn_in_progress_{true};
    int8_t direction_; 

    // Functions
    void publishSpeed(const double linear_speed, double angular_speed);
    void publishState(const std::string& state);


};   // class CameraNode

}   // namespace AGV