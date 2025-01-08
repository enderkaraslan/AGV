#include "rclcpp/rclcpp.hpp"

#include "CameraNode.h"
#include "LidarNode.h"
#include "Driver.h"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto camera_node = std::make_shared<AGV::CameraNode>();
    auto lidar_node = std::make_shared<AGV::LidarNode>();
    auto driver_node = std::make_shared<AGV::Driver>();
    
    rclcpp::executors::MultiThreadedExecutor executor;
    
    executor.add_node(camera_node);
    executor.add_node(lidar_node);
    executor.add_node(driver_node);

    executor.spin();
    
    rclcpp::shutdown();
    
    return 0;
}
