#include <memory>
#include <string>

#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/image.hpp>

class ImageSubscriber : public rclcpp::Node
{
public:
    ImageSubscriber()
        : Node("image_subscriber")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) { handle_image(image); });
        RCLCPP_INFO(this->get_logger(), "Subscribing to '%s'", topic_name_.c_str());
    }

private:
    void handle_image(const sensor_msgs::msg::Image::ConstSharedPtr & image) const
    {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            5000,
            "Received %ux%u %s image (%zu bytes)",
            image->width,
            image->height,
            image->encoding.c_str(),
            image->data.size());
    }

    std::string topic_name_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageSubscriber>());
    rclcpp::shutdown();
    return 0;
}
