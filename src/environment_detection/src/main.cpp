#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "cv_bridge/cv_bridge.h"
#include <opencv2/opencv.hpp>
#include <thread>
#include <mutex>
#include <atomic>
#include <condition_variable>

// Sabitleri namespace içinde tanımlamak daha iyi bir uygulamadır.
namespace Constants {
    constexpr char CameraTopic[]    = "/camera/image_raw";
    constexpr char TwistTopic[]     = "/cmd_vel"; 
    constexpr int QueueSize = 10;
    constexpr double ThresholdValue = 20.0;
    constexpr double LinearSpeedMax = 0.3;
    constexpr double AngularSpeedMultiplier = 0.01;
    constexpr double MinExtent = 0.2;
    constexpr int SleepTimeMs = 10;
    constexpr int CircleRadius = 7;
    constexpr int MiddleCircleRadius = 3;
}

class AgvNode : public rclcpp::Node
{
public:
    enum class State
    {
        NOT_INITIALISED,
        INITIALISED,
        FOLLOW_LINE,
        TURN_LEFT,
        TURN_RIGHT,
        STOP,
        ERROR
    };

    AgvNode() :
        Node("agv_node"), state_{State::INITIALISED}, processing_thread_running_{true}
    {
        camera_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            Constants::CameraTopic,
            Constants::QueueSize,
            [this](const sensor_msgs::msg::Image::SharedPtr msg) { image_callback(msg); });

        velocity_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(
            Constants::TwistTopic,
            Constants::QueueSize);

        processing_thread_ = std::thread(&AgvNode::process_images, this);

        RCLCPP_INFO(this->get_logger(), "AGV node initialized and listening to camera topic.");
    }

    ~AgvNode()
    {
        stop_processing_thread();
    }

    State get_state_copy() const { return state_.load(); }

private:
    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg);
    void process_images();
    void send_velocity_command(double linear_x, double angular_z);
    cv::Point get_contour_center(const std::vector<cv::Point> &contour) const;
    double get_contour_extent(const std::vector<cv::Point> &contour) const;
    void state_machine();
    void stop_processing_thread();

    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr camera_subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr velocity_publisher_;

    std::atomic<State> state_{State::NOT_INITIALISED};
    std::thread processing_thread_;
    std::atomic<bool> processing_thread_running_{false};
    std::mutex image_mutex_;

    cv::Mat cv_image_;
};

void AgvNode::stop_processing_thread()
{
    processing_thread_running_ = false;
    if (processing_thread_.joinable())
    {
        processing_thread_.join();
    }
    RCLCPP_INFO(this->get_logger(), "Processing thread stopped.");
}

void AgvNode::image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
{
    try
    {
        std::lock_guard<std::mutex> lock(image_mutex_);
        cv_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    catch (const cv_bridge::Exception &e)
    {
        RCLCPP_ERROR(this->get_logger(), "Could not convert from '%s' to 'bgr8'.", msg->encoding.c_str());
        state_ = State::ERROR;
    }
}

void AgvNode::process_images()
{
    while (processing_thread_running_)
    {
        cv::Mat local_image;

        // Görüntü kopyalama (mutex ile koruma)
        {
            std::lock_guard<std::mutex> lock(image_mutex_);
            if (cv_image_.empty())
            {
                send_velocity_command(0, 0);
                state_ = State::ERROR;
                RCLCPP_INFO(this->get_logger(), "Could not received an image.");
                std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SleepTimeMs));
                continue;
            }
            local_image = cv_image_.clone();
        }

        // Görüntü işleme
        cv::Mat gray_image, threshold_image;
        std::vector<std::vector<cv::Point>> contours;

        cv::cvtColor(local_image, gray_image, cv::COLOR_BGR2GRAY);
        cv::threshold(gray_image, threshold_image, Constants::ThresholdValue, 255.0, cv::THRESH_BINARY_INV);
        cv::findContours(threshold_image, contours, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

        if (!contours.empty())
        {
            // En büyük konturu bulma
            auto main_contour = *std::max_element(contours.begin(), contours.end(),
                                                  [](const std::vector<cv::Point> &a, const std::vector<cv::Point> &b)
                                                  { return cv::contourArea(a) < cv::contourArea(b); });

            // Kontur merkezini ve genişlik oranını hesaplama
            int width       = local_image.cols;
            int height      = local_image.rows;
            int middle_x    = width / 2;
            int middle_y    = height / 2;
            int mid_pixel   = (int)threshold_image.at<uchar>(middle_y, middle_x);
            int contour_center_x = get_contour_center(main_contour).x;
            double extent = get_contour_extent(main_contour);
            double linear_speed = extent < Constants::MinExtent ? 0.0 : Constants::LinearSpeedMax;
            double angular_speed = extent < Constants::MinExtent ? 0.0 : (middle_x - contour_center_x) * Constants::AngularSpeedMultiplier;
            cv::Rect bounding_rect = cv::boundingRect(main_contour);
            cv::Point topLeft = bounding_rect.tl();
            cv::Point topRight = cv::Point(bounding_rect.x + bounding_rect.width, bounding_rect.y);
            cv::Point bottomLeft = cv::Point(bounding_rect.x, bounding_rect.y + bounding_rect.height);
            cv::Point bottomRight = bounding_rect.br();
            std::cout << "Bounding Rect Köşe Noktaları:" << std::endl;
            std::cout << "Sol Alt (x, y): (" << topLeft.x << ", " << topLeft.y << ")" << std::endl;
            std::cout << "Sağ Alt (x, y): (" << topRight.x << ", " << topRight.y << ")" << std::endl;
            //std::cout << "Sol Üst (x, y): (" << bottomLeft.x << ", " << bottomLeft.y << ")" << std::endl;
            //std::cout << "Sağ Üst (x, y): (" << bottomRight.x << ", " << bottomRight.y << ")" << std::endl;
            cv::Mat part1, part2, part3;

            // İlk Parça: 0 dan partHeight'e kadar
            part1 = threshold_image(cv::Range(479, 480), cv::Range(0, 1)); //cv::Range::all() bütün kolonları al demek
            // İkinci Parça: partHeight'den partHeight * 2 e kadar
            part2 = threshold_image(cv::Range(479, 480), cv::Range(639, 640)); //cv::Range::all() bütün kolonları al demek
            int black_count_1 = cv::countNonZero(part1);
            int black_count_2 = cv::countNonZero(part2);

            if (black_count_1 && mid_pixel == 255)
            {   
                state_ = State::STOP;
                send_velocity_command(0, 0);
                std::cout << "SOLA DONUS" << "\n";
            }
            if (black_count_2 && mid_pixel == 255)
            {   
                state_ = State::STOP;
                send_velocity_command(0, 0);
                std::cout << "SAGA DONUS" << "\n";
            }
            else
                state_ = State::FOLLOW_LINE;
                send_velocity_command(linear_speed, angular_speed);

           
            // Görselleştirme
            cv::drawContours(local_image, std::vector<std::vector<cv::Point>>{main_contour}, -1, cv::Scalar(0, 255, 0), 3);
            cv::circle(local_image, cv::Point(contour_center_x, middle_y), Constants::CircleRadius, cv::Scalar(255, 255, 255), -1);
            cv::circle(local_image, cv::Point(middle_x, middle_y), Constants::MiddleCircleRadius, cv::Scalar(0, 0, 255), -1);

            cv::imshow("Processed Image", local_image);
            cv::waitKey(1);
        }
        else
        {
            // Kontur yoksa hızı sıfırla
            send_velocity_command(0.0, 0.0);
        }

        // Bir sonraki işleme döngüsü için bekleme
        std::this_thread::sleep_for(std::chrono::milliseconds(Constants::SleepTimeMs));
    }
}

void AgvNode::state_machine()
{
    

}

void AgvNode::send_velocity_command(double linear_x, double angular_z)
{
    auto twist_msg = geometry_msgs::msg::Twist();
    twist_msg.linear.x = linear_x;
    twist_msg.angular.z = angular_z;

    velocity_publisher_->publish(twist_msg);
}

cv::Point AgvNode::get_contour_center(const std::vector<cv::Point> &contour) const
{
    cv::Moments M = cv::moments(contour);
    if (M.m00 == 0)
    {
        return cv::Point(0, 0);
    }
    return cv::Point(static_cast<int>(M.m10 / M.m00), static_cast<int>(M.m01 / M.m00));
}

double AgvNode::get_contour_extent(const std::vector<cv::Point> &contour) const
{
    double area = cv::contourArea(contour);
    cv::Rect bounding_rect = cv::boundingRect(contour);
    double rect_area = static_cast<double>(bounding_rect.width * bounding_rect.height);
    return rect_area > 0 ? (area / rect_area) : 0;
}

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<AgvNode>();

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}