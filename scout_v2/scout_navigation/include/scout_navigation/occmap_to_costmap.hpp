#ifndef SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP
#define SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav2_costmap_2d/costmap_2d.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>

namespace scout_navigation
{

class OccMapToCostmapNode : public rclcpp::Node
{
public:
    OccMapToCostmapNode();

private:
    void occMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);
    void publishCostmap();

    rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr occ_map_sub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
};

} // namespace scout_navigation

#endif // SCOUT_NAVIGATION_OCCMAP_TO_COSTMAP_HPP

