#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageViewer : public rclcpp::Node
{
public:
    ImageViewer()
        : Node("image_viewer")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        window_name_ = this->declare_parameter<std::string>("window_name", "ROS 2 Camera");
        const auto timeout_seconds = this->declare_parameter<double>("image_timeout_sec", 2.0);
        if (!std::isfinite(timeout_seconds) || timeout_seconds <= 0.0) {
            throw std::invalid_argument("image_timeout_sec must be a positive number");
        }
        image_timeout_ = std::chrono::duration_cast<std::chrono::milliseconds>(
            std::chrono::duration<double>(timeout_seconds));

        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) { display_image(image); });
        watchdog_timer_ = this->create_wall_timer(
            std::chrono::milliseconds(250),
            [this]() { close_stale_window(); });

        RCLCPP_INFO(
            this->get_logger(),
            "Displaying images from '%s'; closing the window after %.1f seconds without a frame",
            topic_name_.c_str(),
            timeout_seconds);
    }

    ~ImageViewer() override
    {
        try {
            close_window();
        } catch (...) {
            // Destructors must not throw while ROS 2 shuts down.
        }
    }

private:
    void display_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
    {
        try {
            const auto cv_image = cv_bridge::toCvCopy(
                image, sensor_msgs::image_encodings::BGR8);

            std::lock_guard<std::mutex> lock(window_mutex_);
            if (!window_open_) {
                cv::namedWindow(window_name_, cv::WINDOW_AUTOSIZE);
                window_open_ = true;
            }
            cv::imshow(window_name_, cv_image->image);
            cv::waitKey(1);
            last_image_time_ = std::chrono::steady_clock::now();
        } catch (const cv_bridge::Exception & exception) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Unable to display image: %s",
                exception.what());
        } catch (const cv::Exception & exception) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "OpenCV display error: %s",
                exception.what());
        }
    }

    void close_stale_window()
    {
        std::lock_guard<std::mutex> lock(window_mutex_);
        if (!window_open_ || std::chrono::steady_clock::now() - last_image_time_ < image_timeout_) {
            return;
        }

        cv::destroyWindow(window_name_);
        window_open_ = false;
        RCLCPP_INFO(this->get_logger(), "No image received; closed the display window");
    }

    void close_window()
    {
        std::lock_guard<std::mutex> lock(window_mutex_);
        if (window_open_) {
            cv::destroyWindow(window_name_);
            window_open_ = false;
        }
    }

    std::string topic_name_;
    std::string window_name_;
    std::chrono::milliseconds image_timeout_;
    std::chrono::steady_clock::time_point last_image_time_;
    bool window_open_{false};
    std::mutex window_mutex_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr watchdog_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageViewer>());
    rclcpp::shutdown();
    return 0;
}
