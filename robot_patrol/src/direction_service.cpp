#include "rclcpp/executors/single_threaded_executor.hpp"
#include "rclcpp/rclcpp.hpp"
#include <numeric>
#include <robot_patrol/srv/get_direction.hpp>
// #include "sensor_msgs/msg/laser_scan.hpp"

#include <memory>

using GetDirection = robot_patrol::srv::GetDirection;
using std::placeholders::_1;
using std::placeholders::_2;

using namespace std;

class DirectionService : public rclcpp::Node {
public:
  DirectionService() : Node("direction_service_node") {
    srv_ = create_service<GetDirection>(
        "direction_service",
        std::bind(&DirectionService::getdirection_callback, this, _1, _2));
  }

private:
  rclcpp::Service<GetDirection>::SharedPtr srv_;

  void getdirection_callback(
      const std::shared_ptr<GetDirection::Request> request,
      const std::shared_ptr<GetDirection::Response> response) {

    // Get front 180 of the scan
    const int startIndex = 179, endIndex = 539;
    vector<float> front_180;
    front_180.assign(request->laser_data.ranges.begin() + startIndex,
                     request->laser_data.ranges.begin() + endIndex);

    auto sum_of_subslice = [](const std::vector<float> &vec, size_t start,
                              size_t end) {
      if (start >= vec.size() || end > vec.size() || start > end) {
        // Handle invalid slice bounds
        std::cerr << "Invalid slice bounds" << std::endl;
        return 0.0f; // Return 0.0 in case of an error
      }

      if (start == end) {
        return vec[start]; // Return the value at the start index
      }

      // Use std::accumulate to sum the values in the specified sub-slice
      return std::accumulate(vec.begin() + start, vec.begin() + end + 1, 0.0f);
    };

    // Divide 180 degree into three sections: Left, Front, Right.
    float total_dist_sec_right = sum_of_subslice(front_180, 0, 59);
    float total_dist_sec_center = sum_of_subslice(front_180, 60, 119);
    float total_dist_sec_left = sum_of_subslice(front_180, 120, 179);

    // -compare and output string
    if (total_dist_sec_center >= total_dist_sec_left &&
        total_dist_sec_center >= total_dist_sec_right) {
      response->direction = "forward";

    } else if (total_dist_sec_left >= total_dist_sec_center &&
               total_dist_sec_left >= total_dist_sec_left) {
      response->direction = "left";

    } else if (total_dist_sec_right >= total_dist_sec_center &&
               total_dist_sec_right >= total_dist_sec_left) {
      response->direction = "right";
    }

    cout << "left: " << total_dist_sec_left << endl;
    cout << "center: " << total_dist_sec_center << endl;
    cout << "right: " << total_dist_sec_right << endl;
    cout << "`--------------" << endl;
  }
};

int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DirectionService>());
  rclcpp::shutdown();
  return 0;
}