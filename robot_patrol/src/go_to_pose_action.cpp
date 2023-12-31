#include <cmath>
#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "sensor_msgs/msg/laser_scan.hpp"
#include <robot_patrol/action/go_to_pose.hpp>
#include <robot_patrol/srv/get_direction.hpp>

using namespace std;

class GoToPoseActionServer : public rclcpp::Node {
public:
  using GoToPose = robot_patrol::action::GoToPose;
  using GoalHandleMove = rclcpp_action::ServerGoalHandle<GoToPose>;

  explicit GoToPoseActionServer(
      const rclcpp::NodeOptions &options = rclcpp::NodeOptions())
      : Node("go_to_pose_action_server_node", options) {
    using namespace std::placeholders;

    // init callback groups
    callback_group_1 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_2 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);
    callback_group_3 = this->create_callback_group(
        rclcpp::CallbackGroupType::MutuallyExclusive);

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;
    // init subscription option
    rclcpp::SubscriptionOptions option2;
    option1.callback_group = callback_group_2;

    // init odom sub
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&GoToPoseActionServer::odom_callback, this, _1),
        option1);

    // init laser subscription
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&GoToPoseActionServer::scan_callback, this, _1),
        option2);

    // init command vel pub
    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);

    action_server_ = rclcpp_action::create_server<GoToPose>(
        this, "go_to_pose",
        std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
        std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
        std::bind(&GoToPoseActionServer::handle_accepted, this, _1),
        rcl_action_server_get_default_options(), callback_group_3);
  }

private:
  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;
  rclcpp::CallbackGroup::SharedPtr callback_group_3;

  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  // current pos2d
  geometry_msgs::msg::Pose2D cur_pos;
  sensor_msgs::msg::LaserScan cur_scan;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    cur_scan = *msg;
  }

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {

    // set x,y of the current position
    cur_pos.x = msg->pose.pose.position.x;
    cur_pos.y = msg->pose.pose.position.y;

    const float q_x = msg->pose.pose.orientation.x;
    const float q_y = msg->pose.pose.orientation.y;
    const float q_z = msg->pose.pose.orientation.z;
    const float q_w = msg->pose.pose.orientation.w;

    // Yaw (z-axis rotation)
    float sinYaw = 2.0f * (q_w * q_z + q_x * q_y);
    float cosYaw = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
    float yaw = std::atan2(sinYaw, cosYaw);

    // set theta of current positon
    cur_pos.theta = yaw;
  }

  rclcpp_action::GoalResponse
  handle_goal(const rclcpp_action::GoalUUID &uuid,
              std::shared_ptr<const GoToPose::Goal> goal) {
    RCLCPP_INFO(this->get_logger(),
                "Received goal request with x at %f; y at %f; theta at %f",
                goal->goal_pos.x, goal->goal_pos.y, goal->goal_pos.theta);
    (void)uuid;
    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  rclcpp_action::CancelResponse
  handle_cancel(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Received request to cancel goal");
    (void)goal_handle;
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  void handle_accepted(const std::shared_ptr<GoalHandleMove> goal_handle) {
    using namespace std::placeholders;
    // this needs to return quickly to avoid blocking the executor, so spin up
    // a new thread
    std::thread{std::bind(&GoToPoseActionServer::execute, this, _1),
                goal_handle}
        .detach();
  }

  void execute(const std::shared_ptr<GoalHandleMove> goal_handle) {
    RCLCPP_INFO(this->get_logger(), "Executing goal");
    const auto goal = goal_handle->get_goal();
    auto feedback = std::make_shared<GoToPose::Feedback>();

    auto result = std::make_shared<GoToPose::Result>();
    auto move = geometry_msgs::msg::Twist();

    auto desired_pos = goal->goal_pos;

    const float MAX_LINEAR_SPEED = 0.2;
    const float MAX_ANGULAR_SPEED = 0.5;
    while (rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {
        // stop the robot
        move.linear.x = 0;
        move.angular.z = 0;
        vel_pub_->publish(move);
        // set goal state
        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // calculate turning speed
      float diff_x = desired_pos.x - cur_pos.x,
            diff_y = desired_pos.y - cur_pos.y;
      float dirc = atan2(diff_y, diff_x);

      // Set linear speed
      float abs_dist = sqrt(diff_x * diff_x + diff_y * diff_y);
      move.linear.x = min(MAX_LINEAR_SPEED, abs_dist / 5);

      // calculate true delta
      float turn_delta;
      if (abs_dist < 0.05) {
        turn_delta = cur_pos.theta - desired_pos.theta;
      } else {
        turn_delta = cur_pos.theta - dirc;
      }

      if (abs(turn_delta) > M_PI) {
        turn_delta =
            turn_delta > 0 ? 2 * M_PI - turn_delta : 2 * M_PI + turn_delta;
      }

      // set angular speed
      move.angular.z = min(MAX_ANGULAR_SPEED, turn_delta / -3);

      vel_pub_->publish(move);

      feedback->current_pos = cur_pos;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(),
                  "Publish feedback, reaching goal in %f distancel; %f turn "
                  "delta; linear x "
                  "speed: %f, angular z speed %f.",
                  abs_dist, abs(turn_delta), move.linear.x, move.angular.z);

      // check if goal is compelete (if we are close enough)
      if (abs_dist + abs(cur_pos.theta - desired_pos.theta) < 0.1) {
        break;
      } else {
        this_thread::sleep_for(100ms);
      }
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      move.linear.x = 0.0;
      move.angular.z = 0.0;
      vel_pub_->publish(move);
      goal_handle->succeed(result);
      RCLCPP_INFO(this->get_logger(), "Goal succeeded");
    }
  }

}; // class GoToPoseActionServer

int main(int argc, char **argv) {
  rclcpp::init(argc, argv);

  auto action_server = std::make_shared<GoToPoseActionServer>();

  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(action_server);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}