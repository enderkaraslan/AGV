#pragma once

#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include <opencv2/opencv.hpp>
#include "cv_bridge/cv_bridge.h"

#include "my_robot_interfaces/msg/image_info.hpp"
#include "Constants.h"
#include "ImageProcessor.h"


namespace AGV
{

class CameraNode : public rclcpp::Node
{

public:
    // Constructor and destructor for CameraNode class
    CameraNode();
    ~CameraNode();

private:
    // Camera Message subscription implementation
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;

    // Publish image info message
    rclcpp::Publisher<my_robot_interfaces::msg::ImageInfo>::SharedPtr image_info_publisher_;

    // Callback function for camera data
    void cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg);


};   // class CameraNode

}   // namespace AGV