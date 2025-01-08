#include "CameraNode.h"

namespace AGV
{

CameraNode::CameraNode() : Node("camera_node")
{
    // Create a camera subsystem
    camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
        Constants::CameraTopic, Constants::QueueSize,
        [this](const sensor_msgs::msg::Image::SharedPtr msg) {
            cameraCallback(msg);
        });

    image_info_publisher_ = this->create_publisher<my_robot_interfaces::msg::ImageInfo>(
        Constants::ImageInfoTopic, Constants::QueueSize);
    
    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been started.");
}

CameraNode::~CameraNode()
{
    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been stopped.");
}

// Callback function for camera data
void CameraNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg)
{


    cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
    ImageProcessor::ContourAnalysisResult result = ImageProcessor::process(image);


    if (result.valid)
    {
        auto message                    = my_robot_interfaces::msg::ImageInfo();

        message.valid                   = true;
        
        message.contour_center.x        = result.contour_center.x;
        message.contour_center.y        = result.contour_center.y;

        message.extent                  = result.extent;
        message.area                    = result.area;
        message.left_black_pixel_count  = result.left_black_pixel_count;
        message.right_black_pixel_count = result.right_black_pixel_count;
        message.mid_pixel               = result.mid_pixel;
        message.width                   = result.width;
        message.height                  = result.height;
        message.middle_x                = result.middle_x;
        message.middle_y                = result.middle_y;

        image_info_publisher_->publish(message);
        RCLCPP_DEBUG(this->get_logger(), "Image processed successfully.");
    }
    else 
    {
        auto message                    = my_robot_interfaces::msg::ImageInfo();

        message.valid                   = false;

        image_info_publisher_->publish(message);

        RCLCPP_DEBUG(this->get_logger(), "Invalid contour detected. Skipping image processing.");
    }

    
}

} // namespace AGV
