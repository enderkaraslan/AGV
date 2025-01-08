#include "Driver.h"

namespace AGV
{
Driver::Driver() : Node("driver_node")
{

    rclcpp::SubscriptionOptions sub_options_image_info;
    image_info_callback_group_              = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    
    sub_options_image_info.callback_group   = image_info_callback_group_;

    image_info_subscriber_                  = this->create_subscription<my_robot_interfaces::msg::ImageInfo>(
                                                Constants::ImageInfoTopic, Constants::QueueSize,
                                                [this](const my_robot_interfaces::msg::ImageInfo::SharedPtr msg) {
                                                    imageInfoCallback(msg);
                                                },sub_options_image_info);


    rclcpp::SubscriptionOptions sub_options_lidar_alert;
    lidar_alert_callback_group_             = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_lidar_alert.callback_group  = lidar_alert_callback_group_;

    lidar_alert_subscriber_                 = this->create_subscription<my_robot_interfaces::msg::LidarAlert>(
                                                Constants::LidarAlertTopic, Constants::QueueSize,
                                                [this](const my_robot_interfaces::msg::LidarAlert::SharedPtr msg) {
                                                    lidarAlertCallback(msg);
                                                },sub_options_lidar_alert);


    rclcpp::SubscriptionOptions sub_options_odometry;
    odometry_callback_group_                = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_odometry.callback_group     = odometry_callback_group_;

    odometry_subscriber_                  = this->create_subscription<nav_msgs::msg::Odometry>(
                                                Constants::OdometryTopic, Constants::QueueSize,
                                                [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                                                    odometryCallback(msg);
                                                },sub_options_odometry);
    
}

Driver::~Driver()
{

    RCLCPP_DEBUG(this->get_logger(), "DriverNode has been stopped.");
}

void Driver::imageInfoCallback(const my_robot_interfaces::msg::ImageInfo::SharedPtr& msg)
{   
    bool obstacle_detected;
    {
        std::lock_guard<std::mutex> guard(my_mutex_);
        obstacle_detected = is_obstacle_detected;
    }
    
    if (!obstacle_detected && !my_route_.isEmpty())
    {
        double yaw_z;
        {
            std::lock_guard<std::mutex> guard(my_mutex_);
            yaw_z = yaw_z_;
        }

        if (turning_obstacle_detected_)
        {
            my_timer_.start();
            turning_obstacle_detected_ = false;
        }

        if (my_timer_.isRunning())
        {
            double remain_distance = Constants::Distance - distance_travelled_;
            if (remain_distance >= 0)
            {
                motor_.setLinearSpeed(Constants::LinearSpeed);
                motor_.setAngularSpeed(0.0); 
                motor_.publishSpeed();

                auto current_time = my_timer_.elapsedMilliseconds();    
                double elapsed_seconds = current_time / 1000.0;              
                distance_travelled_ = Constants::LinearSpeed * elapsed_seconds; 
                
                {
                    std::lock_guard<std::mutex> guard(my_mutex_);

                    initial_yaw_ = yaw_z;
                }


            }
            else if (std::abs(yaw_z - initial_yaw_) < 1.57)
            {
                motor_.setLinearSpeed(0.0);
                motor_.setAngularSpeed(direction_*Constants::TurningSpeed);
                motor_.publishSpeed();

            }
            else
            {
                distance_travelled_ = 0.0;
                turning_obstacle_detected_ = false;
                my_route_.removeCurrentTurn();
                my_timer_.reset();
            }

        }
        else
        {
            // Çizgi varsa
            if (msg->valid && msg->extent > Constants::MinExtent)
            {
                //  Eğer dönüş varsa
                if (msg->area > Constants::MinimumArea && msg->mid_pixel == Constants::MidPixelValue && (msg->left_black_pixel_count == Constants::BlackPixelValue || msg->right_black_pixel_count == Constants::BlackPixelValue) && is_turn_in_progress_)
                {
                    if (my_route_.isTurn())
                    {
                        if (my_route_.getCurrentTurnType() == Route::TurnType::LEFT)
                        {
                            direction_ = 1;
                        }
                        else if (my_route_.getCurrentTurnType() == Route::TurnType::RIGHT)
                        {
                            direction_ = -1;
                        }
                        my_timer_.start();
                    }
                    else
                    {
                        my_route_.getCurrentTurnOrder()--;
                    }
                    is_turn_in_progress_ = false;
                }

                else if (msg->area > Constants::MinimumArea && msg->mid_pixel == Constants::MidPixelValue)
                {
                    motor_.setLinearSpeed(Constants::LinearSpeed);
                    motor_.setAngularSpeed(0.0);
                    motor_.publishSpeed();
                }
                // Eğer dönüş yoksa
                else
                {
                    is_turn_in_progress_ = true;
                    motor_.setLinearSpeed(Constants::LinearSpeed);
                    motor_.setAngularSpeed(static_cast<double>(msg->middle_x - msg->contour_center.x) * Constants::AngularSpeedScale);  
                    motor_.publishSpeed();

                }

            }

            // Eğer çizgi yoksa boş alandaysa
            else
            {   
                motor_.setLinearSpeed(0.0);
                motor_.setAngularSpeed(0.0);  
                motor_.publishSpeed();

                RCLCPP_DEBUG(this->get_logger(), "Invalid image info detected.");
                
            }
        }
    }
    else
    {
        if (my_timer_.isRunning())
        {
            my_timer_.stop();
            turning_obstacle_detected_ = true;
        }
        motor_.setLinearSpeed(0.0);
        motor_.setAngularSpeed(0.0);  
        motor_.publishSpeed();

        RCLCPP_DEBUG(this->get_logger(), "Obstacle detected.");

    }
    
}   

void Driver::lidarAlertCallback(const my_robot_interfaces::msg::LidarAlert::SharedPtr& msg)
{
    std::lock_guard<std::mutex> guard(my_mutex_);
    is_obstacle_detected = msg->is_obstacle_detect;
}

void Driver::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
{
    double x = msg->pose.pose.orientation.x;
    double y = msg->pose.pose.orientation.y;
    double z = msg->pose.pose.orientation.z;
    double w = msg->pose.pose.orientation.w;
    std::lock_guard<std::mutex> guard(my_mutex_);
    yaw_z_ = eulerFromQuaternion(x, y, z, w);
}

double Driver::eulerFromQuaternion(double x, double y, double z, double w) 
{
    // Roll (x-axis rotation)
    double t0 = +2.0 * (w * x + y * z);
    double t1 = +1.0 - 2.0 * (x * x + y * y);
    double roll_x = std::atan2(t0, t1);

    // Pitch (y-axis rotation)
    double t2 = +2.0 * (w * y - z * x);
    t2 = std::clamp(t2, -1.0, +1.0); 
    double pitch_y = std::asin(t2);

    // Yaw (z-axis rotation)
    double t3 = +2.0 * (w * z + x * y);
    double t4 = +1.0 - 2.0 * (y * y + z * z);
    double yaw_z = std::atan2(t3, t4);

    //double yaw_degrees = yaw_z * (180.0 / M_PI);

    return yaw_z; 
}

} // end namespace