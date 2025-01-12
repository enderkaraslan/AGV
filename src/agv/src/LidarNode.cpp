#include "LidarNode.h"

namespace AGV
{
    //Constructor function
    LidarNode::LidarNode() : Node("lidar_node")
    {
        // Lidar Subscriber
        rclcpp::SubscriptionOptions lidar_callback_options;
        lidar_callback_group_                       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        lidar_callback_options.callback_group       = lidar_callback_group_;

        lidar_subscriber_                           = this->create_subscription<sensor_msgs::msg::LaserScan>(
                                                        Constants::LidarTopic, Constants::QueueSize,
                                                        [this](const sensor_msgs::msg::LaserScan::SharedPtr msg) {
                                                            this->lidarCallback(msg);
                                                        }, lidar_callback_options);


        // State SUbsubscriber
        rclcpp::SubscriptionOptions state_callback_options;
        state_callback_group_                       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
        state_callback_options.callback_group       = state_callback_group_;

        state_subscriber_                           = this->create_subscription<std_msgs::msg::String>(
                                                        Constants::StateTopic, Constants::QueueSize,
                                                        [this](const std_msgs::msg::String::SharedPtr msg) {
                                                            this->stateCallback(msg);
                                                        }, state_callback_options);




        // State Publisher
        state_publisher_                    = this->create_publisher<std_msgs::msg::String>(
                                                Constants::StateTopic, Constants::QueueSize);

        // Speed Publisher
        speed_publisher_                    = this->create_publisher<geometry_msgs::msg::Twist>(
                                                Constants::TwistTopic, Constants::QueueSize);

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
        std::string state;
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state = state_;
        }

        for (size_t i = 0; i < msg->ranges.size(); ++i)
        {
            // If an obstacle is detected within the detection distance, publish the alert message and return
            if (msg->ranges[i] < Constants::DetectionDistance)
            {
                RCLCPP_DEBUG(this->get_logger(), "Obstacle Detected: Distance = %.2f", msg->ranges[i]);
                
                publishState("Obstacle");

                publishSpeed(0.0, 0.0);
                

                return; 
            }
        }

        // Eğer nesne algılandıktan sonra başka hiç bir state e girmediyse ve nesne artık algılanmıyorsa LineFollow state i çalışabilir.
        if (state_ == "Obstacle")
        {

            publishState("LineFollow");
        }

        RCLCPP_DEBUG(this->get_logger(), "No obstacle detected");
    }

    

    void LidarNode::stateCallback(const std_msgs::msg::String::SharedPtr& msg)
    {
        {
            std::lock_guard<std::mutex> lock(state_mutex_);
            state_ = msg->data;
        }

    }

    void LidarNode::publishState(const std::string& state)
    {
        auto message        = std_msgs::msg::String();
        message.data        = state;  

        state_publisher_->publish(message);
    }

    void LidarNode::publishSpeed(const double linear_speed, double angular_speed)
    {
        geometry_msgs::msg::Twist twist;
        twist.linear.x      = linear_speed;
        twist.angular.z     = angular_speed;

        speed_publisher_->publish(twist);
    }
}   // namespace AGV