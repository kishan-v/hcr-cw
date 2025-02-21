
#include <unistd.h>
#include <cmath>

#include "rclcpp/rclcpp.hpp"
#include "unitree_go/msg/sport_mode_state.hpp"

#include "unitree_api/msg/request.hpp"
#include "common/ros2_sport_client.h"

#include "geometry_msgs/msg/twist.hpp"

using namespace std::chrono_literals;

using std::placeholders::_1;
// Create a soprt_request class for soprt commond request
class soprt_request : public rclcpp::Node
{
public:
    soprt_request() : Node("req_sender")
    {
        cmd_vel_suber_ = this->create_subscription<geometry_msgs::msg::Twist>(
            "cmd_vel", 10, std::bind(&soprt_request::cmd_vel_callback, this, _1));

        // the req_puber is set to subscribe "/api/sport/request" topic with dt
        req_puber = this->create_publisher<unitree_api::msg::Request>("/api/sport/request", 10);

        last_twist_ = geometry_msgs::msg::Twist();
    };

private:

    void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg) {
        last_twist_ = *msg;
        std::cout << "GOT CMD VEL" << std::endl;

        std::cout << "Linear: x=" << msg->linear.x << ", y=" << msg->linear.y << ", z=" << msg->linear.z << std::endl;
        std::cout << "Angular: x=" << msg->angular.x << ", y=" << msg->angular.y << ", z=" << msg->angular.z << std::endl;
        // std::cout << "Type: " << typeid(msg->linear.x).name() << std::endl;
        sport_req.Move(req, msg->linear.x, msg->linear.y, msg->angular.z);
        req_puber->publish(req);
    }

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_suber_;

    rclcpp::TimerBase::SharedPtr timer_; // ROS2 timer
    rclcpp::Publisher<unitree_api::msg::Request>::SharedPtr req_puber;

    unitree_api::msg::Request req; // Unitree Go2 ROS2 request message
    SportClient sport_req;
    geometry_msgs::msg::Twist last_twist_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv); // Initialize rclcpp
    
    rclcpp::spin(std::make_shared<soprt_request>()); //Run ROS2 node

    rclcpp::shutdown();
    return 0;
}
