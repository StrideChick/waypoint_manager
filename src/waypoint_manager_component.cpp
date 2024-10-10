#include <waypoint_manager/waypoint_manager_component.hpp>

namespace waypoint_manager
{
WaypointManager::WaypointManager(const rclcpp::NodeOptions & options)
: Node("waypoint_navigator", options)
{ 
  this->client_ = this->create_client<chick_nav_msgs::srv::NavigateToGoal>("waypoint");
  if (!this->client_->wait_for_service(std::chrono::seconds(10))) {
      RCLCPP_ERROR(this->get_logger(), "Service not available after waiting");
      return;
  }

  nav_sub_ = this->create_subscription<std_msgs::msg::Empty>(
    "start_topic", 10, std::bind(&WaypointManager::start_navigation, this,std::placeholders::_1));
}

void WaypointManager::start_navigation(const std_msgs::msg::Empty::SharedPtr msg)
{
  size_t waypoint_index_ = 0;
  if (!loadWaypointsFromFile("/home/suke/chicken_core/src/planner/waypoint_manager/config/waypoint.csv")) {
    RCLCPP_ERROR(this->get_logger(), "Failed to load waypoints from file.");
    return;
  } 
  if (!waypoints_.empty()) {
    auto request = std::make_shared<chick_nav_msgs::srv::NavigateToGoal::Request>();
    request->pose = waypoints_[waypoint_index_];
    RCLCPP_INFO(this->get_logger(), "Navigating to waypoint %d...", waypoint_index_);

    using ServiceResponseFuture = rclcpp::Client<chick_nav_msgs::srv::NavigateToGoal>::SharedFuture;
    auto response_received_callback = [this](ServiceResponseFuture future) {
      // auto result = future.get();
      // path_ = result->path;
    };
    auto result = client_->async_send_request(request, response_received_callback);
  } else {
    RCLCPP_ERROR(this->get_logger(), "No waypoints available to navigate.");
  }
}

//csvファイルから座標取得
bool WaypointManager::loadWaypointsFromFile(const std::string &filename)
{
  waypoints_.clear();
  std::ifstream file(filename);
  if (!file.is_open()) {
      RCLCPP_ERROR(this->get_logger(), "Failed to open waypoint file: %s", filename.c_str());
      return false;
  }
  std::string line;
  bool first_line = true;
  while (std::getline(file, line)) {
      if (first_line) {
          // 最初の行（ヘッダー）をスキップ
          first_line = false;
          continue;
      }

      std::stringstream ss(line);
      std::string x_str, y_str, theta_str;

      std::getline(ss, x_str, ',');
      std::getline(ss, y_str, ',');
      std::getline(ss, theta_str, ',');

      float x = std::stof(x_str);
      float y = std::stof(y_str);
      float theta = std::stof(theta_str);
      waypoints_.push_back(createWaypoint(x, y, theta));
      RCLCPP_INFO(rclcpp::get_logger("WaypointLoader"), "Loaded waypoint: x=%.2f, y=%.2f, theta=%.2f", x, y, theta);
  }
  file.close();
  return true;
}

geometry_msgs::msg::PoseStamped WaypointManager::createWaypoint(float x, float y, float theta)
{
  geometry_msgs::msg::PoseStamped pose;
  pose.header.frame_id = "map";
  pose.header.stamp = this->get_clock()->now();
  pose.pose.position.x = x;
  pose.pose.position.y = y;
  tf2::Quaternion q;
  q.setRPY(0, 0, theta);
  pose.pose.orientation = tf2::toMsg(q);
  return pose;
}
}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(waypoint_manager::WaypointManager)