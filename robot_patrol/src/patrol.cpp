#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("patrol_node") {
    // init callback groups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // init laser subscription
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "/scan", 1, std::bind(&RobotPatrol::scan_callback, this, _1), option1);

    // init pub with Callback group
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
    timer_ = this->create_wall_timer(
        500ms, std::bind(&RobotPatrol::timer_callback, this), callback_group_2);
  }

private:
  void timer_callback() {
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = 0;
    message.angular.z = 0.5;
    vel_pub_->publish(message);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    auto getFront180

        RCLCPP_INFO(this->get_logger(), "lenth of ranges: '%d'",
                    msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "range_max: '%d'", msg->ranges[0]);

    cout << "this is test within scan callback" << endl;
  }

  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // init node
  std::shared_ptr<RobotPatrol> node = std::make_shared<RobotPatrol>();

  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(node);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}
