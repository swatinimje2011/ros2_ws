#include <atomic>
#include <chrono>
#include <condition_variable>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <utility>
#include <vector>

#include <cv_bridge/cv_bridge.hpp>
#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>

#include "ros_camera/webcam_capture.hpp"

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
        : Node("image_publisher")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_frame");
        camera_config_.device_index = this->declare_parameter<int>("camera_index", 0);
        camera_config_.width = this->declare_parameter<int>("width", 640);
        camera_config_.height = this->declare_parameter<int>("height", 480);
        camera_config_.fps = this->declare_parameter<double>("publish_rate_hz", 10.0);

        if (!is_valid(camera_config_)) {
            throw std::invalid_argument(
                "camera_index must be non-negative; width, height, and publish_rate_hz must be positive");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(
            topic_name_, rclcpp::SensorDataQoS());
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                return update_parameters(parameters);
            });
        capture_thread_ = std::thread([this]() { capture_loop(); });

        RCLCPP_INFO(
            this->get_logger(),
            "Capturing camera %d for '%s' (requested %dx%d at %.1f Hz)",
            camera_config_.device_index,
            topic_name_.c_str(),
            camera_config_.width,
            camera_config_.height,
            camera_config_.fps);
    }

    ~ImagePublisher() override
    {
        running_ = false;
        retry_cv_.notify_all();
        if (capture_thread_.joinable()) {
            capture_thread_.join();
        }
    }

private:
    static bool is_valid(const ros_camera::WebcamConfig & config)
    {
        return config.device_index >= 0 && config.width > 0 && config.height > 0 &&
               std::isfinite(config.fps) && config.fps > 0.0;
    }

    rcl_interfaces::msg::SetParametersResult update_parameters(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        ros_camera::WebcamConfig proposed_config;
        std::string proposed_frame_id;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            proposed_config = camera_config_;
            proposed_frame_id = frame_id_;
        }

        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "camera_index") {
                if (!is_integer(parameter)) {
                    return rejected("camera_index must be a non-negative integer");
                }
                proposed_config.device_index = static_cast<int>(parameter.as_int());
            } else if (parameter.get_name() == "width") {
                if (!is_integer(parameter)) {
                    return rejected("width must be a positive integer");
                }
                proposed_config.width = static_cast<int>(parameter.as_int());
            } else if (parameter.get_name() == "height") {
                if (!is_integer(parameter)) {
                    return rejected("height must be a positive integer");
                }
                proposed_config.height = static_cast<int>(parameter.as_int());
            } else if (parameter.get_name() == "publish_rate_hz") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    return rejected("publish_rate_hz must be a positive number");
                }
                proposed_config.fps = parameter.as_double();
            } else if (parameter.get_name() == "frame_id") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_STRING) {
                    return rejected("frame_id must be a string");
                }
                proposed_frame_id = parameter.as_string();
            } else if (parameter.get_name() == "topic_name") {
                return rejected("topic_name cannot be changed while the node is running");
            }
        }

        if (!is_valid(proposed_config)) {
            return rejected(
                "camera_index must be non-negative; width, height, and publish_rate_hz must be positive");
        }

        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            reconfigure_requested_ = proposed_config.device_index != camera_config_.device_index ||
                                    proposed_config.width != camera_config_.width ||
                                    proposed_config.height != camera_config_.height ||
                                    proposed_config.fps != camera_config_.fps;
            camera_config_ = proposed_config;
            frame_id_ = std::move(proposed_frame_id);
        }
        retry_cv_.notify_all();

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    static bool is_integer(const rclcpp::Parameter & parameter)
    {
        return parameter.get_type() == rclcpp::ParameterType::PARAMETER_INTEGER &&
               parameter.as_int() <= std::numeric_limits<int>::max() &&
               parameter.as_int() >= std::numeric_limits<int>::min();
    }

    rcl_interfaces::msg::SetParametersResult rejected(const std::string & reason) const
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = reason;
        return result;
    }

    void capture_loop()
    {
        while (rclcpp::ok() && running_) {
            ros_camera::WebcamConfig config;
            bool reconfigure_requested;
            {
                std::lock_guard<std::mutex> lock(config_mutex_);
                config = camera_config_;
                reconfigure_requested = reconfigure_requested_;
                reconfigure_requested_ = false;
            }

            if (reconfigure_requested || !capture_.is_open()) {
                if (!capture_.open(config)) {
                    RCLCPP_WARN(
                        this->get_logger(),
                        "Unable to open camera %d; retrying in one second",
                        config.device_index);
                    wait_for_retry();
                    continue;
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "Camera opened at %dx%d, %.1f Hz",
                    capture_.actual_width(),
                    capture_.actual_height(),
                    capture_.actual_fps());
            }

            cv::Mat frame;
            if (!capture_.read(frame)) {
                RCLCPP_WARN(this->get_logger(), "Failed to read a camera frame; reopening the camera");
                capture_.close();
                wait_for_retry();
                continue;
            }

            publish_frame(frame);
        }

        capture_.close();
    }

    void wait_for_retry()
    {
        std::unique_lock<std::mutex> lock(config_mutex_);
        retry_cv_.wait_for(lock, std::chrono::seconds(1), [this]() {
            return !running_ || reconfigure_requested_;
        });
    }

    void publish_frame(const cv::Mat & frame)
    {
        std::string frame_id;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            frame_id = frame_id_;
        }

        if (frame.type() != CV_8UC3) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(),
                *this->get_clock(),
                5000,
                "Unsupported camera frame type %d; expected BGR8",
                frame.type());
            return;
        }

        cv_bridge::CvImage cv_image;
        cv_image.header.stamp = this->now();
        cv_image.header.frame_id = frame_id;
        cv_image.encoding = sensor_msgs::image_encodings::BGR8;
        cv_image.image = frame;
        publisher_->publish(*cv_image.toImageMsg());
    }

    std::string topic_name_;
    std::string frame_id_;
    ros_camera::WebcamConfig camera_config_;
    bool reconfigure_requested_{true};
    std::mutex config_mutex_;
    std::condition_variable retry_cv_;
    std::atomic<bool> running_{true};
    ros_camera::WebcamCapture capture_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
    std::thread capture_thread_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
