#include "rclcpp/rclcpp.hpp"
#include "linux/can.h"
#include "linux/can/raw.h"
#include "sys/socket.h"
#include "unistd.h"
#include "iostream"
#include "cstring"
#include "functional"
#include "sys/ioctl.h"
#include "net/if.h"
#include "cstdlib"

class CANNode : public rclcpp::Node
{
public:
    CANNode() : Node("can_communication_node")
    {
        if (system("sudo ip link set can0 up type can bitrate 1000000") != 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to set up CAN interface - Make sure to have the CAN module plugged in!");
            return;
        }

        RCLCPP_INFO(this->get_logger(), "Started CAN node");

        // Create a CAN socket
        socket_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
        if (socket_fd_ < 0) {
            RCLCPP_ERROR(this->get_logger(), "Failed to create CAN socket");
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Created CAN socket");
        }

        // Specify the CAN interface
        struct ifreq ifr;
        strcpy(ifr.ifr_name, "can0");
        ioctl(socket_fd_, SIOCGIFINDEX, &ifr);

        // Bind the socket to the CAN interface
        struct sockaddr_can addr;
        addr.can_family = AF_CAN;
        addr.can_ifindex = ifr.ifr_ifindex;
        if (bind(socket_fd_, (struct sockaddr *)&addr, sizeof(addr)) < 0){
            RCLCPP_ERROR(this->get_logger(), "Failed to bind CAN socket");
            close(socket_fd_);
            return;
        } else {
            RCLCPP_INFO(this->get_logger(), "Binded to CAN socket to interface");
        }

        timer_ = this->create_wall_timer(
            std::chrono::seconds(1),
            std::bind(&CANNode::send_periodic_message, this));
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

    void send_periodic_message()
    {
        uint32_t speedControl = 900000;
        send_can_message(speedControl);
        sleep(5);

        speedControl = 000000;
        send_can_message(speedControl);
        sleep(5);

        speedControl = -900000;
        send_can_message(speedControl);
        sleep(5);

        speedControl = 000000;
        send_can_message(speedControl);
        sleep(5);
    }

    void send_can_message(uint32_t speedControl)
    {
        struct can_frame frame;
        frame.can_id = 0x141;   // CAN ID 0x141 for RMD-X motor
        frame.can_dlc = 8;      // Data length code

        frame.data[0] = 0xA2;                           // Command byte
        frame.data[1] = 0x00;                           // NULL
        frame.data[2] = 0x00;                           // NULL
        frame.data[3] = 0x00;                           // NULL
        frame.data[4] = (uint8_t)(speedControl);        // Speed control low
        frame.data[5] = (uint8_t)(speedControl >> 8);   // Speed control
        frame.data[6] = (uint8_t)(speedControl >> 16);  // Speed control
        frame.data[7] = (uint8_t)(speedControl >> 24);  // Speed control high

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