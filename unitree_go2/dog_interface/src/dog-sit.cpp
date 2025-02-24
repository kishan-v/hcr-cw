#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "geometry_msgs/msg/twist.hpp"

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp

    auto node = rclcpp::Node::make_shared("req_sender");

    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;
    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;

    req_puber = node->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);
    
    sleep(2); // Wait for initialisation or this doesn't work
    
    sport_req.Sit(req);
    req_puber->publish(req);
    
    sleep(2);

    rclcpp::shutdown();
    return 0;
}
