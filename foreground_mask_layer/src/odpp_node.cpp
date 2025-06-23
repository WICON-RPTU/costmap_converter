#include <rclcpp/rclcpp.hpp>
#include <nav2_costmap_2d/costmap_2d_ros.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <vector>
#include <cmath>
#include <algorithm>

class DynamicObstacleNode : public rclcpp::Node {
public:
    DynamicObstacleNode()
        : Node("dynamic_obstacle_node")
    {
        // Initialize costmap
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

        // Timer to process costmap
        timer_ = this->create_wall_timer(
            std::chrono::milliseconds(200),
            std::bind(&DynamicObstacleNode::processCostmap, this));

        // Publisher for visualization
        marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>("dynamic_obstacle_markers", 10);

        RCLCPP_INFO(this->get_logger(), "DynamicObstacleNode started.");
    }

private:
    std::shared_ptr<nav2_costmap_2d::Costmap2DROS> costmap_ros_;
    std::unique_ptr<std::thread> costmap_thread_;
    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;
    int previous_marker_count_ = 0;

    void processCostmap() {
        auto costmap = costmap_ros_->getCostmap();
        unsigned char cost_threshold = nav2_costmap_2d::INSCRIBED_INFLATED_OBSTACLE;
        unsigned int size_x = costmap->getSizeInCellsX();
        unsigned int size_y = costmap->getSizeInCellsY();

        // 2D visited array to track processed cells
        std::vector<std::vector<bool>> visited(size_x, std::vector<bool>(size_y, false));

        // Structure to hold clusters
        using Cell = std::pair<unsigned int, unsigned int>;
        std::vector<std::vector<geometry_msgs::msg::Point>> clusters;

        // Flood-fill to group boundary points into clusters
        auto floodFill = [&](unsigned int i, unsigned int j, std::vector<geometry_msgs::msg::Point> &cluster) {
            std::vector<Cell> stack = {{i, j}};
            while (!stack.empty()) {
                auto [ci, cj] = stack.back();
                stack.pop_back();

                if (visited[ci][cj]) continue;
                visited[ci][cj] = true;

                if (costmap->getCost(ci, cj) >= cost_threshold) {
                    double wx, wy;
                    costmap->mapToWorld(ci, cj, wx, wy);
                    geometry_msgs::msg::Point p;
                    p.x = wx;
                    p.y = wy;
                    p.z = 0.0;
                    cluster.push_back(p);

                    // Check 8-connected neighbors
                    for (int di = -1; di <= 1; ++di) {
                        for (int dj = -1; dj <= 1; ++dj) {
                            int ni = ci + di;
                            int nj = cj + dj;
                            if (ni >= 0 && nj >= 0 && ni < static_cast<int>(size_x) && nj < static_cast<int>(size_y) &&
                                !visited[ni][nj]) {
                                stack.push_back({ni, nj});
                            }
                        }
                    }
                }
            }
        };

        // Iterate over all cells and group boundary points into clusters
        for (unsigned int i = 0; i < size_x; ++i) {
            for (unsigned int j = 0; j < size_y; ++j) {
                if (!visited[i][j] && costmap->getCost(i, j) >= cost_threshold) {
                    std::vector<geometry_msgs::msg::Point> cluster;
                    floodFill(i, j, cluster);
                    if (!cluster.empty()) {
                        clusters.push_back(cluster);
                    }
                }
            }
        }

        // Prepare marker array for visualization
        visualization_msgs::msg::MarkerArray marker_array;

        // Clear old markers
        for (int id = 0; id < previous_marker_count_; ++id) {
            visualization_msgs::msg::Marker delete_marker;
            delete_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
            delete_marker.header.stamp = now();
            delete_marker.ns = "boundary";
            delete_marker.id = id;
            delete_marker.action = visualization_msgs::msg::Marker::DELETE;
            marker_array.markers.push_back(delete_marker);
        }

        // Add new markers
        int marker_id = 0;
        for (const auto &cluster : clusters) {
            // Compute convex hull for each cluster
            std::vector<geometry_msgs::msg::Point> hull = computeConvexHull(cluster);

            // Create a marker to display the convex hull as a closed polygon
            visualization_msgs::msg::Marker boundary_marker;
            boundary_marker.header.frame_id = costmap_ros_->getGlobalFrameID();
            boundary_marker.header.stamp = now();
            boundary_marker.ns = "boundary";
            boundary_marker.id = marker_id++;
            boundary_marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
            boundary_marker.action = visualization_msgs::msg::Marker::ADD;
            boundary_marker.scale.x = 0.05; // Line width
            boundary_marker.color.r = 1.0;
            boundary_marker.color.g = 0.0;
            boundary_marker.color.b = 0.0;
            boundary_marker.color.a = 1.0;

            for (const auto &pt : hull) {
                boundary_marker.points.push_back(pt);
            }
            if (!hull.empty()) {
                // Close the polygon
                boundary_marker.points.push_back(hull.front());
            }

            marker_array.markers.push_back(boundary_marker);
        }

        // Update the count of markers
        previous_marker_count_ = marker_id;

        marker_pub_->publish(marker_array);
    }

    // Convex hull computation (integrated as a private method)
    std::vector<geometry_msgs::msg::Point> computeConvexHull(const std::vector<geometry_msgs::msg::Point> &points) {
        std::vector<geometry_msgs::msg::Point> hull;
        if (points.empty()) return hull;

        size_t leftmost = 0;
        for (size_t i = 1; i < points.size(); i++) {
            if (points[i].x < points[leftmost].x ||
                (points[i].x == points[leftmost].x && points[i].y < points[leftmost].y))
                leftmost = i;
        }

        size_t p = leftmost, q;
        do {
            hull.push_back(points[p]);
            q = (p + 1) % points.size();
            for (size_t i = 0; i < points.size(); i++) {
                if (cross(points[p], points[i], points[q]) > 0)
                    q = i;
            }
            p = q;
        } while (p != leftmost);

        return hull;
    }

    // Helper: cross product of OA x OB
    double cross(const geometry_msgs::msg::Point &O, const geometry_msgs::msg::Point &A, const geometry_msgs::msg::Point &B) {
        return (A.x - O.x) * (B.y - O.y) - (A.y - O.y) * (B.x - O.x);
    }
};

int main(int argc, char **argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<DynamicObstacleNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}