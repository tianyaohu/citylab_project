#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <chrono>
#include <cmath>
#include <functional>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class RobotPatrol : public rclcpp::Node {
public:
  RobotPatrol() : Node("patrol_node"), linear_x(0.1) {
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
    message.linear.x = linear_x;
    message.angular.z = angular_z;
    vel_pub_->publish(message);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {

    RCLCPP_INFO(this->get_logger(), "lenth of ranges: '%d'",
                msg->ranges.size());
    RCLCPP_INFO(this->get_logger(), "range_max: '%f'", msg->ranges[0]);

    // Lambda function to filter out inf values and find the index of the
    // maximum value
    auto findMaxIndex = [](const std::vector<float> &arr) {
      // Create a copy of the array to avoid modifying the original
      std::vector<float> filteredArray(arr);

      // Remove inf values from the array
      filteredArray.erase(
          std::remove_if(filteredArray.begin(), filteredArray.end(),
                         [](float value) { return std::isinf(value); }),
          filteredArray.end());

      // Find the iterator to the maximum element
      auto maxIterator =
          std::max_element(filteredArray.begin(), filteredArray.end());

      // Calculate the index of the maximum element
      int index = std::distance(filteredArray.begin(), maxIterator);

      return index;
    };

    direction_ = findMaxIndex(msg->ranges) / 720.0 * M_PI - M_PI / 2;

    // max value in array
    RCLCPP_INFO(this->get_logger(), "direction_ is : '%f'", direction_);
  }

  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  float direction_;
  float linear_x;
  float angular_z;
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
