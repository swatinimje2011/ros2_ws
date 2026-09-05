#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <utility>
#include <vector>

#include <rcl_interfaces/msg/set_parameters_result.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImagePublisher : public rclcpp::Node
{
public:
    ImagePublisher()
        : Node("image_publisher")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        frame_id_ = this->declare_parameter<std::string>("frame_id", "camera_frame");
        width_ = this->declare_parameter<int>("width", 640);
        height_ = this->declare_parameter<int>("height", 480);
        publish_rate_hz_ = this->declare_parameter<double>("publish_rate_hz", 10.0);

        if (width_ <= 0 || height_ <= 0 || !std::isfinite(publish_rate_hz_) || publish_rate_hz_ <= 0.0) {
            throw std::invalid_argument("width, height, and publish_rate_hz must be positive");
        }

        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(topic_name_, 10);
        recreate_timer(publish_rate_hz_);
        parameter_callback_handle_ = this->add_on_set_parameters_callback(
            [this](const std::vector<rclcpp::Parameter> & parameters) {
                return update_parameters(parameters);
            });

        RCLCPP_INFO(
            this->get_logger(),
            "Publishing placeholder RGB8 images on '%s' at %.1f Hz",
            topic_name_.c_str(),
            publish_rate_hz_);
    }

private:
    rcl_interfaces::msg::SetParametersResult update_parameters(
        const std::vector<rclcpp::Parameter> & parameters)
    {
        int proposed_width;
        int proposed_height;
        double proposed_publish_rate_hz;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            proposed_width = width_;
            proposed_height = height_;
            proposed_publish_rate_hz = publish_rate_hz_;
        }

        for (const auto & parameter : parameters) {
            if (parameter.get_name() == "width") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                    parameter.as_int() > std::numeric_limits<int>::max()) {
                    return rejected("width must be a positive integer");
                }
                proposed_width = static_cast<int>(parameter.as_int());
            } else if (parameter.get_name() == "height") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_INTEGER ||
                    parameter.as_int() > std::numeric_limits<int>::max()) {
                    return rejected("height must be a positive integer");
                }
                proposed_height = static_cast<int>(parameter.as_int());
            } else if (parameter.get_name() == "publish_rate_hz") {
                if (parameter.get_type() != rclcpp::ParameterType::PARAMETER_DOUBLE) {
                    return rejected("publish_rate_hz must be a positive number");
                }
                proposed_publish_rate_hz = parameter.as_double();
            }
        }

        if (proposed_width <= 0 || proposed_height <= 0 ||
            !std::isfinite(proposed_publish_rate_hz) || proposed_publish_rate_hz <= 0.0) {
            return rejected("width, height, and publish_rate_hz must be positive and valid");
        }

        const auto period = std::chrono::duration<double>(1.0 / proposed_publish_rate_hz);
        if (std::chrono::duration_cast<std::chrono::nanoseconds>(period).count() <= 0) {
            return rejected("publish_rate_hz is too high");
        }

        std::lock_guard<std::mutex> lock(config_mutex_);
        const bool rate_changed = proposed_publish_rate_hz != publish_rate_hz_;
        width_ = proposed_width;
        height_ = proposed_height;
        publish_rate_hz_ = proposed_publish_rate_hz;
        if (rate_changed) {
            recreate_timer(publish_rate_hz_);
        }

        rcl_interfaces::msg::SetParametersResult result;
        result.successful = true;
        return result;
    }

    rcl_interfaces::msg::SetParametersResult rejected(const std::string & reason) const
    {
        rcl_interfaces::msg::SetParametersResult result;
        result.successful = false;
        result.reason = reason;
        return result;
    }

    void recreate_timer(double publish_rate_hz)
    {
        const auto period = std::chrono::duration<double>(1.0 / publish_rate_hz);
        timer_ = this->create_wall_timer(
            std::chrono::duration_cast<std::chrono::nanoseconds>(period),
            [this]() { publish_image(); });
    }

    void publish_image()
    {
        int width;
        int height;
        {
            std::lock_guard<std::mutex> lock(config_mutex_);
            width = width_;
            height = height_;
        }

        sensor_msgs::msg::Image image;
        image.header.stamp = this->now();
        image.header.frame_id = frame_id_;
        image.height = static_cast<std::uint32_t>(height);
        image.width = static_cast<std::uint32_t>(width);
        image.encoding = "rgb8";
        image.is_bigendian = false;
        image.step = static_cast<std::uint32_t>(width * 3);
        image.data.resize(static_cast<std::size_t>(height) * image.step);
        std::fill(image.data.begin(), image.data.end(), 0U);
        publisher_->publish(std::move(image));
    }

    std::string topic_name_;
    std::string frame_id_;
    int width_;
    int height_;
    double publish_rate_hz_;
    std::mutex config_mutex_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::node_interfaces::OnSetParametersCallbackHandle::SharedPtr parameter_callback_handle_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImagePublisher>());
    rclcpp::shutdown();
    return 0;
}
