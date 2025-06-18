#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <thread>

class DynamicObstacleNode : public rclcpp::Node {
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node")
    {
        // Spin up costmap with your custom plugin
        costmap_ros_ = std::make_shared<nav2_costmap_2d::Costmap2DROS>(
            "dynamic_obstacle_costmap", std::string{get_namespace()}, "dynamic_obstacle_costmap");

        costmap_thread_ = std::make_unique<std::thread>(
            [](rclcpp_lifecycle::LifecycleNode::SharedPtr node) {
                rclcpp::spin(node->get_node_base_interface());
            },
            costmap_ros_);

        rclcpp_lifecycle::State state;
        costmap_ros_->on_configure(state);
        costmap_ros_->on_activate(state);

        // Timer to process costmap and run blob detection/tracking
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        // Publisher for visualization (optional)
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_obstacle_markers", 10);

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode started.");
    }

    void processCostmap() {
        auto costmap = costmap_ros_->getCostmap();
        // Example: Iterate over costmap and extract foreground mask
        // TODO: Replace with your blob detection and tracking logic

        // Placeholder: visualize all lethal cells
        visualization_msgs::msg::MarkerArray marker_array;
        visualization_msgs::msg::Marker marker;
        marker.header.frame_id = costmap_ros_->getGlobalFrameID();
        marker.header.stamp = now();
        marker.ns = "foreground";
        marker.type = visualization_msgs::msg::Marker::POINTS;
        marker.action = visualization_msgs::msg::Marker::ADD;
        marker.scale.x = 0.1;
        marker.scale.y = 0.1;
        marker.color.g = 1.0;
        marker.color.a = 1.0;

        for (unsigned int i = 0; i < costmap->getSizeInCellsX(); ++i) {
            for (unsigned int j = 0; j < costmap->getSizeInCellsY(); ++j) {
                if (costmap->getCost(i, j) == nav2_costmap_2d::LETHAL_OBSTACLE) {
                    geometry_msgs::msg::Point p;
                    double wx, wy;
                    costmap->mapToWorld(i, j, wx, wy);
                    p.x = wx;
                    p.y = wy;
                    marker.points.push_back(p);
                }
            }
        }
        marker_array.markers.push_back(marker);
        marker_pub_->publish(marker_array);

        // TODO: Add blob detection and tracking here
    }

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}