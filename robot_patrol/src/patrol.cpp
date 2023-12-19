#include "rclcpp/rclcpp.hpp"
#include "rclcpp/utilities.hpp"
#include "sensor_msgs/msg/detail/laser_scan__struct.hpp"
#include <algorithm>
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
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 1);
    timer_ = this->create_wall_timer(
        100ms, std::bind(&RobotPatrol::timer_callback, this), callback_group_2);
  }

private:
  void timer_callback() {
    // cout << "linear x is " << linear_x << endl;
    // cout << "angular z is " << angular_z << endl;
    auto message = geometry_msgs::msg::Twist();
    message.linear.x = linear_x;
    message.angular.z = angular_z;
    vel_pub_->publish(message);
  }

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    // Lambda function to filter out inf values and find the index of the
    // maximum value
    auto findIndex = [](const std::vector<float> &arr,
                        const int startIndex = 179, const int endIndex = 539,
                        bool findMax = true) {
      // to igonore ranges beyong laser capacity
      int max_laser_capacity = 30;

      // Create a copy of the array to avoid modifying the original
      std::vector<float> filteredArray;
      copy(arr.begin() + startIndex, arr.begin() + endIndex,
           back_inserter(filteredArray));

      //   for (size_t i = 0; i < filteredArray.size(); i++) {
      //     cout << "i is " << i << " and reading is " << filteredArray[i] <<
      //     endl;
      //   }

      //   cout << "right?" << filteredArray[0] << endl;

      //   cout << "Center?" << filteredArray[180] << endl;

      //   cout << "left?" << filteredArray[359] << endl;

      // Remove inf values from the array
      filteredArray.erase(std::remove_if(filteredArray.begin(),
                                         filteredArray.end(),
                                         [max_laser_capacity](float value) {
                                           return value > max_laser_capacity;
                                         }),
                          filteredArray.end());

      // Find the iterator to the minimum or maximum element based on the
      // predicate
      auto iterator =
          findMax
              ? std::max_element(filteredArray.begin(), filteredArray.end())
              : std::min_element(filteredArray.begin(), filteredArray.end());

      // Calculate the index of the maximum element
      int index = std::distance(filteredArray.begin(), iterator);

      return index;
    };

    // find the max index in the front of the robot
    int temp = findIndex(msg->ranges);
    float half_range = 180.0;

    direction_ = (temp - half_range) / half_range * (M_PI / 2);

    // set special protocol when something is in the front
    // The Philosophy here is to have a wide front sensing range white keeping
    // the sensitive low (aka: trigering distnace lowe)
    int index_center = 359, vision_width = 90;
    float avoid_dist = 0.17;
    int min_index_within_vision =
        findIndex(msg->ranges, index_center - vision_width,
                  index_center + vision_width, false);
    float min_within_vision =
        msg->ranges[index_center - vision_width + min_index_within_vision];

    cout << "min within vision: " << min_within_vision << endl;

    if (min_within_vision < avoid_dist) { // this finds the min within range
      linear_x = 0;
      if (last_direction > 0) {

        angular_z = -0.7;
        RCLCPP_INFO(this->get_logger(), "Avoiding obstacles: turning RIGHT!");

      } else {
        angular_z = 0.7;
        RCLCPP_INFO(this->get_logger(), "Avoiding obstacles: turning LEFT!");
      }
      // force sleep

      rclcpp::sleep_for(100ms);
    } else {
      angular_z = direction_ / 2;
      // angular_z = 0.1;
      linear_x = 0.1;
      last_direction = direction_;
    }

    // testing laser scan direction
    //  // back
    //  cout << "0 lazer reading " << msg->ranges[0] << endl;
    //  // right
    //  cout << "179 lazer reading " << msg->ranges[179] << endl;
    //  // front
    //  cout << "360 lazer reading " << msg->ranges[360] << endl;
    //  // left
    //  cout << "540 lazer reading " << msg->ranges[540] << endl;
    //  // back
    //  cout << "719 lazer reading " << msg->ranges[719] << endl;

    RCLCPP_INFO(this->get_logger(), "index is : '%d'", temp);
  }

  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  rclcpp::TimerBase::SharedPtr timer_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;

  float last_direction;
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
