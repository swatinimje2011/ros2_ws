#include <rclcpp/rclcpp.hpp>

class CameraNode : public rclcpp::Node
{
public:
    CameraNode()
        : Node("camera_node")
    {
        RCLCPP_INFO(this->get_logger(), "Hello ROS 2!");
    }
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);

    auto node = std::make_shared<CameraNode>();

    rclcpp::spin(node);

    rclcpp::shutdown();

    return 0;
}