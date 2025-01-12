#include "CameraNode.h"

namespace AGV
{

CameraNode::CameraNode() : Node("camera_node")
{
    // Camera Subscriber
    rclcpp::SubscriptionOptions camera_callback_options;
    camera_callback_group_                      = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    camera_callback_options.callback_group      = camera_callback_group_;

    camera_subscriber_                          = this->create_subscription<sensor_msgs::msg::Image>(
                                                    Constants::CameraTopic, Constants::QueueSize,
                                                    [this](const sensor_msgs::msg::Image::SharedPtr msg) {
                                                        cameraCallback(msg);
                                                    }, camera_callback_options);

    // State Subscriber
    rclcpp::SubscriptionOptions state_callback_options;
    state_callback_group_                       = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    state_callback_options.callback_group       = state_callback_group_;

    state_subscriber_                           = this->create_subscription<std_msgs::msg::String>(
                                                    Constants::StateTopic, Constants::QueueSize,
                                                    [this](const std_msgs::msg::String::SharedPtr msg) {
                                                        this->stateCallback(msg);
                                                    }, state_callback_options);

    // State Publisher
    state_publisher_                            = this->create_publisher<std_msgs::msg::String>(
                                                    Constants::StateTopic, Constants::QueueSize);

    // Speed Publisher
    speed_publisher_                            = this->create_publisher<geometry_msgs::msg::Twist>(
                                                    Constants::TwistTopic, Constants::QueueSize);

    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been started.");
}

CameraNode::~CameraNode()
{
    RCLCPP_DEBUG(this->get_logger(), "CameraNode has been stopped.");
}

// Callback function for camera data
void CameraNode::cameraCallback(const sensor_msgs::msg::Image::SharedPtr& msg)
{

    std::string state; 
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state = state_;
    }

    if (state == "LineFollow")
    {
        cv::Mat image = cv_bridge::toCvCopy(msg, "bgr8")->image;
        ImageProcessor::ContourAnalysisResult result = ImageProcessor::process(image);

        if (result.valid && result.extent > Constants::MinExtent)
        {
            // Dönüş alanları
            if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue && (result.left_black_pixel_count == Constants::BlackPixelValue || result.right_black_pixel_count == Constants::BlackPixelValue) && is_turn_in_progress_)
            {
                // Eğer bu dönüşten dönmemiz gerekiyorsa sağ mu sol mu olduğunu kontrol et
                if (my_route_.isTurn())
                {
                    if (my_route_.getCurrentTurnType() == Route::TurnType::LEFT)
                    {
                        publishState("TurnLeft");
                    }
                    else if (my_route_.getCurrentTurnType() == Route::TurnType::RIGHT)
                    {
                        publishState("TurnRight");
                    }
                    my_timer_.start();
                }
                else if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue)
                {
        
                    publishSpeed(Constants::LinearSpeed, 0.0);
                }
                else
                {
                    my_route_.getCurrentTurnOrder()--;
                }
                is_turn_in_progress_ = false;
            }
            // Dönüş alanına gelirken sağa sola kaymaması için düz devam ediyor
            else if (result.area > Constants::MinimumArea && result.mid_pixel == Constants::MidPixelValue)
            {
        
                publishSpeed(Constants::LinearSpeed, 0.0);

            }
            // Dönüş yok çizgiyi takip ediyoruz
            else
            {
                is_turn_in_progress_ = true;

                publishSpeed(Constants::LinearSpeed, static_cast<double> ((result.middle_x - result.contour_center.x) * Constants::AngularSpeedScale));

            }

        }
        // Eğer çizgi yoksa duruyoruz
        else
        {
            publishSpeed(0.0, 0.0);
        }
    }

    else if (state == "TurnLeft")
    {
        //TODO: TURN LEFT ALGORİTHM
    }

}

void CameraNode::stateCallback(const std_msgs::msg::String::SharedPtr& msg)
{
    {
        std::lock_guard<std::mutex> lock(state_mutex_);
        state_ = msg->data;
    }

}

void CameraNode::publishSpeed(const double linear_speed, double angular_speed)
{
    geometry_msgs::msg::Twist twist;
    twist.linear.x      = linear_speed;
    twist.angular.z     = angular_speed;

    speed_publisher_->publish(twist);
}

void CameraNode::publishState(const std::string& state)
{
    auto message        = std_msgs::msg::String();
    message.data        = state;  

    state_publisher_->publish(message);
}

} // namespace AGV
