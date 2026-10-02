#include <chrono>
#include <memory>
#include <string>

#include <cv_bridge/cv_bridge.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/image_encodings.hpp>
#include <sensor_msgs/msg/image.hpp>
#include <std_msgs/msg/float64.hpp>

class ImageProcessingNode : public rclcpp::Node
{
public:
    ImageProcessingNode()
        : Node("imageProcessing_node")
    {
        topic_name_ = this->declare_parameter<std::string>("topic_name", "camera/image_raw");
        window_name_ = this->declare_parameter<std::string>("window_name", "Processed image");
        processing_time_topic_ = this->declare_parameter<std::string>(
            "processing_time_topic", "image_processing/processing_time_ms");

        processing_time_publisher_ = this->create_publisher<std_msgs::msg::Float64>(
            processing_time_topic_, 10);
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            topic_name_, rclcpp::SensorDataQoS(),
            [this](sensor_msgs::msg::Image::ConstSharedPtr image) { process_image(image); });
        RCLCPP_INFO(
            this->get_logger(),
            "Applying Canny edge detection to '%s'; publishing processing time on '%s'",
            topic_name_.c_str(), processing_time_topic_.c_str());
    }

    ~ImageProcessingNode() override
    {
        try {
            cv::destroyWindow(window_name_);
        } catch (...) {
            // Destructors must not throw during ROS shutdown.
        }
    }

private:
    void process_image(const sensor_msgs::msg::Image::ConstSharedPtr & image)
    {
        try {
            const auto processing_start = std::chrono::steady_clock::now();
            const auto input = cv_bridge::toCvCopy(
                image, sensor_msgs::image_encodings::BGR8);
            cv::Mat grayscale;
            cv::Mat blurred;
            cv::Mat edges;
            cv::cvtColor(input->image, grayscale, cv::COLOR_BGR2GRAY);
            cv::GaussianBlur(grayscale, blurred, cv::Size(5, 5), 0.0);
            cv::Canny(blurred, edges, 50.0, 150.0);

            const auto elapsed = std::chrono::duration<double, std::milli>(
                std::chrono::steady_clock::now() - processing_start);
            std_msgs::msg::Float64 processing_time;
            processing_time.data = elapsed.count();
            processing_time_publisher_->publish(processing_time);

            cv::imshow(window_name_, edges);
            cv::waitKey(1);
        } catch (const cv_bridge::Exception & exception) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Unable to convert camera image: %s", exception.what());
        } catch (const cv::Exception & exception) {
            RCLCPP_ERROR_THROTTLE(
                this->get_logger(), *this->get_clock(), 5000,
                "Image processing or display failed: %s", exception.what());
        }
    }

    std::string topic_name_;
    std::string window_name_;
    std::string processing_time_topic_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr processing_time_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessingNode>());
    rclcpp::shutdown();
    return 0;
}
