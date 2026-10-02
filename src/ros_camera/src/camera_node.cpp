#include <chrono>
#include <memory>
#include <stdexcept>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_camera/webcam_capture.hpp"

using namespace std::chrono_literals;

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
        : Node("camera_node")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_frame");
        camera_config_.device_index = this->declare_parameter<int>("camera_index", 0);
        camera_config_.width = this->declare_parameter<int>("width", 0);
        camera_config_.height = this->declare_parameter<int>("height", 0);
        camera_config_.fps = this->declare_parameter<double>("publish_rate_hz", 10.0);

        if (camera_config_.device_index < 0 || camera_config_.width < 0 ||
            camera_config_.height < 0 || camera_config_.fps <= 0.0) {
            throw std::invalid_argument(
                "camera_index, width, and height must be non-negative; publish_rate_hz must be positive");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_name_, rclcpp::SensorDataQoS());
        capture_timer_ = this->create_wall_timer(100ms, [this]() { capture_and_publish(); });
        RCLCPP_INFO(this->get_logger(), "Publishing camera images on '%s'", topic_name_.c_str());
    }

private:
    void capture_and_publish()
    {
        if (!capture_.is_open()) {
            if (!capture_.open(camera_config_)) {
                RCLCPP_WARN_THROTTLE(
                    this->get_logger(), *this->get_clock(), 5000,
                    "Unable to open camera %d; retrying", camera_config_.device_index);
                return;
            }
        }

        cv::Mat frame;
        if (!capture_.read(frame)) {
            RCLCPP_WARN_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Failed to read a camera frame; reopening the camera");
            capture_.close();
            return;
        }

        if (frame.type() != CV_8UC3) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Unsupported camera frame type %d; expected BGR8", frame.type());
            return;
        }

        cv_bridge::CvImage image;
        image.header.stamp = this->now();
        image.header.frame_id = frame_id_;
        image.encoding = sensor_msgs::image_encodings::BGR8;
        image.image = frame;
        publisher_->publish(*image.toImageMsg());
    }

    std::string topic_name_;
    std::string frame_id_;
    ros_camera::WebcamConfig camera_config_;
    ros_camera::WebcamCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr capture_timer_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}
