#include "rclcpp/rclcpp.hpp"
#include "linux/can.h"
#include "linux/can/raw.h"
#include "sys/socket.h"
#include "unistd.h"
#include "iostream"
#include "cstring"

class CANNode : public rclcpp::Node
{
public:
    CANNode() : Node("can_communication_node")
    {
        RCLCPP_INFO(this->get_logger(), "Started CANNode");

        // Create a CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
            return;
        }

        // Specify the CAN interface
        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        ioctl(socket_fd_, SIOCGIFINDEX, &ifr);

        // Bind the socket to the CAN interface
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, (struct sockaddr *)&addr), sizeof(addr) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
            close(socket_fd_);
            return;
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CANNode::timerCallback, this));
            std::bind(&CANNode::send_can_message, this));
    }

    ~CANNode()
    {
        close(socket_fd_);
    }

private:
    void timerCallback()
    {
        RCLCPP_INFO(this->get_logger(), "ping!");
    }

    void send_can_message()
    {
        struct can_frame frame;
        frame.can_id = 0x141;
        frame.can_dlc = 1;
        frame.data[0] = 0xAB;

        if (write(socket_fd_, &frame, sizeof(frame)) != sizeof(frame)){
            RCLCPP_ERROR(this->get_logger(), "Failed to send CAN message");
        } else {
            RCLCPP_INFO(this->get_logger(), "Sent CAN message");
        }
    }

    int socket_fd_;
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