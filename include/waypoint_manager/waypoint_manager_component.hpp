#ifndef WAYPOINT_MANAGER_HPP_
#define WAYPOINT_MANAGER_HPP_

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <std_msgs/msg/empty.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <string>
#include <fstream>
#include <sstream>
#include "chick_nav_msgs/srv/navigate_to_goal.hpp"

namespace waypoint_manager
{
class WaypointManager : public rclcpp::Node
{
public:
    explicit WaypointManager(const rclcpp::NodeOptions & options);

private:
    geometry_msgs::msg::PoseStamped createWaypoint(float x, float y, float theta);
    void start_navigation(const std_msgs::msg::Empty::SharedPtr msg);
    void sendNextGoal();
    bool loadWaypointsFromFile(const std::string &file_path);
    
    //ros
    rclcpp::Subscription<std_msgs::msg::Empty>::SharedPtr nav_sub_;
    rclcpp::Client<chick_nav_msgs::srv::NavigateToGoal>::SharedPtr client_;
    std::vector<geometry_msgs::msg::PoseStamped> waypoints_;
    // void handleServiceResponse(const rclcpp::Client<nav2_msgs::srv::NavigateToPose>::SharedFuture future);
};
}

#endif  // WAYPOINT_NAVIGATOR_HPP_
