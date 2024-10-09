#include <waypoint_manager/waypoint_manager_component.hpp>
#include <memory>

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions options;
  auto component = std::make_shared<waypoint_manager::WaypointManager>(options);
  rclcpp::spin(component);
  rclcpp::shutdown();
  return 0;
}