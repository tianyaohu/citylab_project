#include "rclcpp/rclcpp.hpp"
#include <chrono>
#include <geometry_msgs/msg/twist.hpp>

using namespace std::chrono_literals;

class PublishAge : public rclcpp::Node {
public:
  PublishAge() : Node("patrol_node") {
    publisher_ =
        this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&PublishAge::timer_callback, this));
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0;
    message.angular.z = 0.5;
    publisher_->publish(message);
  }

  // attributes
  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<PublishAge>());
  rclcpp::shutdown();
  return 0;
}
