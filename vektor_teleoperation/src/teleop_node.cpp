#include <chrono>
#include <functional>
#include <memory>
#include <string>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "sensor_msgs/msg/joy.hpp"

class TeleopJoy : public rclcpp::Node
{
public:
  TeleopJoy() : Node("teleop_joy")
  {
    cmd_vel_pub_ =
      this->create_publisher<geometry_msgs::msg::Twist>(
        "/cmd_vel",
        rclcpp::QoS(10));
        
    joy_sub_ =
      this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        rclcpp::QoS(10),
        std::bind(&TeleopJoy::teleop_joy_callback, this, std::placeholders::_1));
  }

private:
  void teleop_joy_callback(const sensor_msgs::msg::Joy &joy_msg)
  {
    // joy_msg.buttons[0]
    // joy_node.axis[4]
    cmd_vel_msg_.linear.x = joy_msg.axes[4];
    cmd_vel_msg_.angular.z = joy_msg.axes[3];

    cmd_vel_pub_->publish(cmd_vel_msg_);
  }

  geometry_msgs::msg::Twist cmd_vel_msg_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<TeleopJoy>());
  rclcpp::shutdown();
  return 0;
}
