#include "rclcpp/rclcpp.hpp"
#include "rclcpp/timer.hpp"
#include <robot_patrol/srv/get_direction.hpp>

#include <chrono>
#include <cstdlib>
#include <future>
#include <memory>

using std::placeholders::_1;

using namespace std::chrono_literals;
using namespace std;

class ServiceClient : public rclcpp::Node {
public:
  ServiceClient() : Node("test_client_node") {
    // init laser subscription
    laser_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
        "scan", 1, std::bind(&ServiceClient::scan_callback, this, _1));

    // init client
    client_ = this->create_client<robot_patrol::srv::GetDirection>(
        "direction_service");
  }

  bool is_service_done() const { return this->service_done_; }

private:
  rclcpp::Client<robot_patrol::srv::GetDirection>::SharedPtr client_;
  rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr laser_sub_;

  bool service_done_ = false;

  void scan_callback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
    cout << "within scan callback" << endl;
    // block to wait for service to become available
    while (!client_->wait_for_service(1s)) {
      if (!rclcpp::ok()) {
        RCLCPP_ERROR(
            this->get_logger(),
            "Client interrupted while waiting for service. Terminating...");
        return;
      }
      RCLCPP_INFO(this->get_logger(),
                  "Service Unavailable. Waiting for Service...");
    }
    // service to became available

    // init request
    auto request = std::make_shared<robot_patrol::srv::GetDirection::Request>();
    request->laser_data = *msg;
    // set service status flag to false;
    service_done_ = false;
    //
    auto result_future = client_->async_send_request(
        request, std::bind(&ServiceClient::response_callback, this,
                           std::placeholders::_1));
  }

  void response_callback(
      rclcpp::Client<robot_patrol::srv::GetDirection>::SharedFuture future) {
    auto status = future.wait_for(1s);
    if (status == std::future_status::ready) {
      RCLCPP_INFO(this->get_logger(), "Result: success %s",
                  future.get()->direction.c_str());
      service_done_ = true;
    } else {
      RCLCPP_INFO(this->get_logger(), "Service In-Progress...");
    }
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);

  std::shared_ptr<ServiceClient> service_client =
      std::make_shared<ServiceClient>();
  // Initialize one MultiThreadedExecutor object
  rclcpp::executors::MultiThreadedExecutor executor;
  executor.add_node(service_client);
  executor.spin();

  rclcpp::shutdown();
  return 0;
}