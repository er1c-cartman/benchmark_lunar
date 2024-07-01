#include "scout_navigation/occmap_to_costmap.hpp"

namespace scout_navigation
{

OccMapToCostmapNode::OccMapToCostmapNode()
    : Node("occmap_to_costmap_node"),
      tf_buffer_(std::make_shared<tf2_ros::Buffer>(this->get_clock())),
      tf_listener_(std::make_shared<tf2_ros::TransformListener>(*tf_buffer_))
{
    costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>("costmap");
    costmap_ros_->on_configure(rclcpp_lifecycle::State());

    occ_map_sub_ = this->create_subscription<nav_msgs::msg::OccupancyGrid>(
        "/traversability_occ_map", rclcpp::SensorDataQoS(),
        std::bind(&OccMapToCostmapNode::occMapCallback, this, std::placeholders::_1));

    costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 10);
}

void OccMapToCostmapNode::occMapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
{
    auto costmap = costmap_ros_->getCostmap();

    unsigned int size_x = msg->info.width;
    unsigned int size_y = msg->info.height;
    double resolution = msg->info.resolution;
    double origin_x = msg->info.origin.position.x;
    double origin_y = msg->info.origin.position.y;

    costmap->resizeMap(size_x, size_y, resolution, origin_x, origin_y);

    for (unsigned int y = 0; y < size_y; ++y)
    {
        for (unsigned int x = 0; x < size_x; ++x)
        {
            int8_t occ_value = msg->data[y * size_x + x];
            unsigned char cost_value = 0;

            if (occ_value == -1)
            {
                cost_value = nav2_costmap_2d::NO_INFORMATION;
            }
            else if (occ_value == 0)
            {
                cost_value = nav2_costmap_2d::FREE_SPACE;
            }
            else if (occ_value == 100)
            {
                cost_value = nav2_costmap_2d::LETHAL_OBSTACLE;
            }
            else
            {
                cost_value = static_cast<unsigned char>((occ_value / 100.0) * 254);
            }

            costmap->setCost(x, y, cost_value);
        }
    }

    publishCostmap();
}

void OccMapToCostmapNode::publishCostmap()
{
    auto costmap = costmap_ros_->getCostmap();
    auto costmap_msg = std::make_shared<nav_msgs::msg::OccupancyGrid>();

    costmap_msg->header.frame_id = "odom";
    costmap_msg->header.stamp = this->get_clock()->now();
    costmap_msg->info.width = costmap->getSizeInCellsX();
    costmap_msg->info.height = costmap->getSizeInCellsY();
    costmap_msg->info.resolution = costmap->getResolution();
    costmap_msg->info.origin.position.x = costmap->getOriginX();
    costmap_msg->info.origin.position.y = costmap->getOriginY();
    costmap_msg->info.origin.position.z = 0.0;
    costmap_msg->info.origin.orientation.w = 1.0;

    costmap_msg->data.resize(costmap->getSizeInCellsX() * costmap->getSizeInCellsY());

    for (unsigned int y = 0; y < costmap->getSizeInCellsY(); ++y)
    {
        for (unsigned int x = 0; x < costmap->getSizeInCellsX(); ++x)
        {
            costmap_msg->data[y * costmap->getSizeInCellsX() + x] = costmap->getCost(x, y);
        }
    }

    costmap_pub_->publish(*costmap_msg);
}

} // namespace scout_navigation

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<scout_navigation::OccMapToCostmapNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}

