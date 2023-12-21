#include <functional>
#include <memory>
#include <thread>

#include "geometry_msgs/msg/detail/pose2_d__struct.hpp"
#include "nav_msgs/msg/detail/odometry__struct.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include <robot_patrol/action/go_to_pose.hpp>

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

    // init subscription option
    rclcpp::SubscriptionOptions option1;
    option1.callback_group = callback_group_1;

    // init odom sub
    odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
        "odom", 1, std::bind(&GoToPoseActionServer::odom_callback, this, _1),
        option1);

    // this->action_server_ = rclcpp_action::create_server<GoToPose>(
    //     this, "go_to_pose",
    //     std::bind(&GoToPoseActionServer::handle_goal, this, _1, _2),
    //     std::bind(&GoToPoseActionServer::handle_cancel, this, _1),
    //     std::bind(&GoToPoseActionServer::handle_accepted, this, _1),
    //     rcl_action_server_get_default_options(), callback_group_2);

    vel_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
  }

private:
  // attributes
  rclcpp::CallbackGroup::SharedPtr callback_group_1;
  rclcpp::CallbackGroup::SharedPtr callback_group_2;

  rclcpp_action::Server<GoToPose>::SharedPtr action_server_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr vel_pub_;
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;

  // current pos2d
  geometry_msgs::msg::Pose2D cur_pos;

  void odom_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
    cout << "within odom callback" << endl;

    cout << "x is " << msg->pose.pose.position.x << endl;
    cout << "y is " << msg->pose.pose.position.y << endl;
    cout << "z is " << msg->pose.pose.position.z << endl;

    // set x,y of the current position
    cur_pos.x = msg->pose.pose.position.x;
    cur_pos.y = msg->pose.pose.position.y;

    float yaw, pitch, roll;

    const float q_x = msg->pose.pose.orientation.x;
    const float q_y = msg->pose.pose.orientation.y;
    const float q_z = msg->pose.pose.orientation.z;
    const float q_w = msg->pose.pose.orientation.w;

    // Roll (x-axis rotation)
    float sinRoll = 2.0f * (q_w * q_x + q_y * q_z);
    float cosRoll = 1.0f - 2.0f * (q_x * q_x + q_y * q_y);
    roll = std::atan2(sinRoll, cosRoll);

    // Pitch (y-axis rotation)
    float sinPitch = 2.0f * (q_w * q_y - q_z * q_x);
    // Avoid gimbal lock at the poles
    if (std::abs(sinPitch) >= 1)
      pitch = std::copysign(M_PI / 2, sinPitch);
    else
      pitch = std::asin(sinPitch);

    // Yaw (z-axis rotation)
    float sinYaw = 2.0f * (q_w * q_z + q_x * q_y);
    float cosYaw = 1.0f - 2.0f * (q_y * q_y + q_z * q_z);
    yaw = std::atan2(sinYaw, cosYaw);

    cout << "row is " << roll << endl;
    cout << "pitch is " << pitch << endl;
    cout << "yaw is " << yaw << endl;

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
    // this needs to return quickly to avoid blocking the executor, so spin up a
    // new thread
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
    rclcpp::Rate loop_rate(1);

    while (rclcpp::ok()) {
      // Check if there is a cancel request
      if (goal_handle->is_canceling()) {

        // stop the robot

        result->status = false;
        goal_handle->canceled(result);
        RCLCPP_INFO(this->get_logger(), "Goal canceled");
        return;
      }

      // send a client call to /direction_service and store the result in a
      // future

      // GoToPose robot forward and send feedback
      //   move.linear.x = 0.3;
      vel_pub_->publish(move);
      feedback->current_pos = cur_pos;
      goal_handle->publish_feedback(feedback);
      RCLCPP_INFO(this->get_logger(), "Publish feedback");

      loop_rate.sleep();
    }

    // Check if goal is done
    if (rclcpp::ok()) {
      result->status = true;
      move.linear.x = 0.0;
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