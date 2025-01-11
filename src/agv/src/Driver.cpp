#include "Driver.h"

namespace AGV
{
Driver::Driver() : Node("driver_node")
{

    rclcpp::SubscriptionOptions sub_options_image_info;
    image_info_callback_group_                  = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_image_info.callback_group       = image_info_callback_group_;

    image_info_subscriber_                      = this->create_subscription<my_robot_interfaces::msg::ImageInfo>(
                                                    Constants::ImageInfoTopic, Constants::QueueSize,
                                                    [this](const my_robot_interfaces::msg::ImageInfo::SharedPtr msg) {
                                                        imageInfoCallback(msg);
                                                    },sub_options_image_info);


    rclcpp::SubscriptionOptions sub_options_lidar_alert;
    lidar_alert_callback_group_                 = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_lidar_alert.callback_group      = lidar_alert_callback_group_;

    lidar_alert_subscriber_                     = this->create_subscription<my_robot_interfaces::msg::LidarAlert>(
                                                    Constants::LidarAlertTopic, Constants::QueueSize,
                                                    [this](const my_robot_interfaces::msg::LidarAlert::SharedPtr msg) {
                                                        lidarAlertCallback(msg);
                                                    },sub_options_lidar_alert);


    rclcpp::SubscriptionOptions sub_options_odometry;
    odometry_callback_group_                    = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_odometry.callback_group         = odometry_callback_group_;

    odometry_subscriber_                        = this->create_subscription<nav_msgs::msg::Odometry>(
                                                    Constants::OdometryTopic, Constants::QueueSize,
                                                    [this](const nav_msgs::msg::Odometry::SharedPtr msg) {
                                                        odometryCallback(msg);
                                                    },sub_options_odometry);
    

    rclcpp::SubscriptionOptions sub_options_occupancy_grid;
    occupancy_grid_callback_group_              = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_occupancy_grid.callback_group   = occupancy_grid_callback_group_;

    occupancy_grid_subscriber_                  = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
                                                    Constants::OccupancyGridTopic, Constants::QueueSize,
                                                    [this](const nav_msgs::msg::OccupancyGrid::SharedPtr msg) {
                                                        occupancyGridCallback(msg);
                                                    },sub_options_occupancy_grid);

    rclcpp::SubscriptionOptions sub_options_path_;
    path_callback_group_                        = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
    sub_options_path_.callback_group           = path_callback_group_;

    path_subscriber_                          = this->create_subscription<nav_msgs::msg::Path>(
                                                    Constants::PathTopic, Constants::QueueSize,
                                                    [this](const nav_msgs::msg::Path::SharedPtr msg) {
                                                        pathCallback(msg);
                                                    },sub_options_path_);


    marker_publisher_                           = this->create_publisher<visualization_msgs::msg::Marker>(
                                                    Constants::MarkerPublish, Constants::QueueSize);

    goal_pose_publisher_                        = this->create_publisher<geometry_msgs::msg::PoseStamped>(
                                                    Constants::GoalPosePublish, Constants::QueueSize);


}

Driver::~Driver()
{

    RCLCPP_DEBUG(this->get_logger(), "DriverNode has been stopped.");
}

void Driver::imageInfoCallback(const my_robot_interfaces::msg::ImageInfo::SharedPtr& msg)
{   

    if (nav2_timer_.isRunning())
    {
        if (nav2_timer_.elapsedMilliseconds() / 1000.0 > 1.5)
        {
            nav2_timer_.reset();
        }
    }
    bool obstacle_detected;
    {
        std::lock_guard<std::mutex> guard(my_mutex_);
        obstacle_detected = is_obstacle_detected;
    }

    if (!obstacle_detected && !my_route_.isEmpty() && !nav2_timer_.isRunning())
    {
        obstacle_detected_flag_ = true; //
        obstacle_detected_timer_.reset();


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
    else if(obstacle_detected && !nav2_timer_.isRunning())
    {
        RCLCPP_DEBUG(this->get_logger(), "Obstacle detected.");
        if (obstacle_detected_flag_)
        {
            std::cout << "Obstacle detected" << std::endl;
            obstacle_detected_timer_.start();
            obstacle_detected_flag_ = false;
        }
        // Eğer dönüş sırasında nesne algılanırsa
        if (my_timer_.isRunning())
        {
            my_timer_.stop();
            turning_obstacle_detected_ = true;
        }

        // standart olarak nesne algılandığında hızı sıfırladık
        motor_.setLinearSpeed(0.0);
        motor_.setAngularSpeed(0.0);  
        motor_.publishSpeed();

        //TODO: Eğer nesne algılandıktan sonra 5 saniye içinde gitmezse kaçış manevrasını başlat...
        if (obstacle_detected_timer_.elapsedMilliseconds() / 1000.0 >= 5.0)
        {
            auto message = geometry_msgs::msg::PoseStamped();
            message.header.stamp = this->get_clock()->now();
            message.header.frame_id = "map";  // Reference frame for the goal

            // Calculate the goal position 1 meter ahead of the robot
            double distance_ahead = 2.0;  // 1 meter
            double goal_x = x_ + distance_ahead * std::cos(yaw_z_);
            double goal_y = y_ + distance_ahead * std::sin(yaw_z_);

            message.pose.position.x = goal_x;
            message.pose.position.y = goal_y;
            message.pose.position.z = 0.0;  // Keep Z at 0 for 2D navigation

            // Maintain the same orientation as the robot
            message.pose.orientation.x = q_x;
            message.pose.orientation.y = q_y;
            message.pose.orientation.z = q_z;  // Quaternion Z component
            message.pose.orientation.w = w;  // Quaternion W component

            RCLCPP_INFO(this->get_logger(), "Publishing goal pose: x=%.2f, y=%.2f",
                        message.pose.position.x, message.pose.position.y);
            goal_pose_publisher_->publish(message);


            obstacle_detected_timer_.reset();
            nav2_timer_.start();

        }
        
    }
    

    
}   

void Driver::lidarAlertCallback(const my_robot_interfaces::msg::LidarAlert::SharedPtr& msg)
{
    std::lock_guard<std::mutex> guard(my_mutex_);
    is_obstacle_detected = msg->is_obstacle_detect;
}

void Driver::odometryCallback(const nav_msgs::msg::Odometry::SharedPtr& msg)
{


    q_x = msg->pose.pose.orientation.x;
    q_y = msg->pose.pose.orientation.y;
    q_z = msg->pose.pose.orientation.z;
    w = msg->pose.pose.orientation.w;
    std::lock_guard<std::mutex> guard(my_mutex_);
    x_ = msg->pose.pose.position.x;  
    y_ = msg->pose.pose.position.y; 
    yaw_z_ = eulerFromQuaternion(q_x, q_y, q_z, w);
    
}

void Driver::occupancyGridCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr& msg)
{
    occupancy_grid_data_.resolution_ = msg->info.resolution;
    occupancy_grid_data_.originX_ = msg->info.origin.position.x;
    occupancy_grid_data_.originY_ = msg->info.origin.position.y;
    occupancy_grid_data_.width_ = msg->info.width;
    occupancy_grid_data_.height_ = msg->info.height;
    occupancy_grid_data_.data_ = msg->data;


    /*auto marker = visualization_msgs::msg::Marker();

    // Header bilgileri
    marker.header.frame_id = "map";  // RViz'deki frame_id
    marker.header.stamp = this->get_clock()->now();

    marker.ns = "example_marker";  // Marker namespace
    marker.id = 0;                 // Marker ID
    marker.type = visualization_msgs::msg::Marker::SPHERE;  // Marker türü
    marker.action = visualization_msgs::msg::Marker::ADD;   // Marker ekleme işlemi


    // Marker pozisyonu
    marker.pose.position.x = x_;
    marker.pose.position.y = y_;
    marker.pose.position.z = 0.0;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    RCLCPP_INFO(this->get_logger(), "X: %f, Y: %f", x_, y_);

    // Marker boyutları
    marker.scale.x = 0.5;
    marker.scale.y = 0.5;
    marker.scale.z = 0.5;

    // Marker renk ayarları (RGBA)
    marker.color.r = 1.0f;
    marker.color.g = 0.0f;
    marker.color.b = 0.0f;
    marker.color.a = 1.0f;

    // Marker ömrü (-1.0, süresiz)
    marker.lifetime = rclcpp::Duration::from_seconds(0);

    marker_publisher_->publish(marker);*/
    
}

void Driver::pathCallback(const nav_msgs::msg::Path::SharedPtr& msg)
{

    


    nav2_timer_.restart();

}


std::vector<std::vector<int8_t>> Driver::costmap(const std::vector<int8_t>& data, uint32_t width, uint32_t height, float resolution, uint8_t expansion_size)
{
    std::vector<std::vector<int8_t>> grid(height, std::vector<int8_t>(width));
    for (int i = 0; i < height; ++i) 
    {
        for (int j = 0; j < width; ++j) 
        {
            grid[i][j] = data[i * width + j];
        }
    }

    std::vector<std::pair<int, int>> wall_positions;
    for (int i = 0; i < height; ++i) 
    {
        for (int j = 0; j < width; ++j) 
        {
            if (grid[i][j] == 100) 
            {
                wall_positions.emplace_back(i, j);
            }
        }
    }

    for (const auto& wall : wall_positions) 
    {
        int wall_x = wall.first;
        int wall_y = wall.second;

        for (int i = -expansion_size; i <= expansion_size; ++i) 
        {
            for (int j = -expansion_size; j <= expansion_size; ++j) 
            {
                int x = wall_x + i;
                int y = wall_y + j;

                if (x >= 0 && x < static_cast<int>(height) && y >= 0 && y < static_cast<int>(width)) 
                {
                    grid[x][y] = 100;
                }
            }
        }
    }

    return grid;
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