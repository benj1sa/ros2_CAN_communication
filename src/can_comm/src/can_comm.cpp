#include "rclcpp/rclcpp.hpp"

class CANNode : public rclcpp::Node
{
public:
    CANNode() : Node("can_communication_node")
    {
        RCLCPP_INFO(this->get_logger(), "Started CANNode");
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&CANNode::timerCallback, this));
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "Hello from ROS2")
    }

    rclcpp::TimerBase::SharedPtr timer_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<CANNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}