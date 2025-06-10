#include "foreground_mask_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace foreground_mask
{

    void ForegroundMaskLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node. Ensure node_ is initialized properly."};
        }

        // Declare plugin parameters
        declareParameter("enabled", rclcpp::ParameterValue(true));
        declareParameter("map_topic", rclcpp::ParameterValue(std::string("/map")));
        declareParameter("inflation_radius", rclcpp::ParameterValue(8)); // Default inflation radius

        node->get_parameter(name_ + "." + "enabled", enabled_);
        node->get_parameter(name_ + "." + "map_topic", map_topic_);
        node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_);

        RCLCPP_INFO(node->get_logger(), "ForegroundMaskLayer initialized with map_topic=%s, inflation_radius=%d",
                    map_topic_.c_str(), inflation_radius_);

        // Subscribe to the static map
        map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic_,
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&ForegroundMaskLayer::incomingStaticMap, this, std::placeholders::_1));

        current_ = true;
    }

    void ForegroundMaskLayer::incomingStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        static_map_ = msg;
        RCLCPP_INFO(logger_, "Static map received (%ux%u)", msg->info.width, msg->info.height);

        // Resize the inflated static map buffer to hold one cell per original map cell.
        inflated_static_.resize(static_map_->info.width * static_map_->info.height, 0);

        // Inflate the static map using the parameter inflation_radius_.
        for (unsigned int y = 0; y < static_map_->info.height; ++y)
        {
            for (unsigned int x = 0; x < static_map_->info.width; ++x)
            {
                unsigned int idx = x + y * static_map_->info.width;
                if (static_map_->data[idx] == 100) // Obstacle cell in static map
                {
                    for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
                    {
                        for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            if (nx >= 0 && ny >= 0 &&
                                nx < static_cast<int>(static_map_->info.width) &&
                                ny < static_cast<int>(static_map_->info.height))
                            {
                                unsigned int inflated_idx = nx + ny * static_map_->info.width;
                                inflated_static_[inflated_idx] = 100; // Mark as inflated obstacle value
                            }
                        }
                    }
                }
            }
        }
        RCLCPP_INFO(logger_, "Static map inflated.");
    }

    void ForegroundMaskLayer::updateBounds(double /*robot_x*/, double /*robot_y*/, double /*robot_yaw*/,
                                           double *min_x, double *min_y, double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        *min_x = -std::numeric_limits<float>::max();
        *min_y = -std::numeric_limits<float>::max();
        *max_x = std::numeric_limits<float>::max();
        *max_y = std::numeric_limits<float>::max();
    }

    void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                          int min_i, int min_j, int max_i, int max_j)
    {
        if (!enabled_ || !static_map_)
        {
            RCLCPP_WARN(logger_, "ForegroundMaskLayer is disabled or static map not available.");
            return;
        }

        unsigned char *master_array = master_grid.getCharMap();

        // Iterate over the costmap bounds
        for (int j = min_j; j < max_j; ++j)
        {
            for (int i = min_i; i < max_i; ++i)
            {
                unsigned int index = master_grid.getIndex(i, j);

                // Transform the costmap cell to the static map frame directly
                double wx, wy;
                master_grid.mapToWorld(i, j, wx, wy);

                // Convert world coordinates to static map indices
                unsigned int mx = static_cast<unsigned int>((wx - static_map_->info.origin.position.x) / static_map_->info.resolution);
                unsigned int my = static_cast<unsigned int>((wy - static_map_->info.origin.position.y) / static_map_->info.resolution);

                if (mx < static_map_->info.width && my < static_map_->info.height)
                {
                    unsigned int inflated_idx = mx + my * static_map_->info.width;

                    unsigned char inflated_val = inflated_static_[inflated_idx];
                    unsigned char master_val = master_array[index];

                    // Comparison logic: foreground detection
                    if (master_val >= 90 && inflated_val < 50) // Detected as dynamic obstacle
                    {
                        master_array[index] = nav2_costmap_2d::LETHAL_OBSTACLE; // Mark as foreground obstacle
                    }
                    else
                    {
                        master_array[index] = nav2_costmap_2d::FREE_SPACE; // Clear static/background data
                    }
                }
                else
                {
                    master_array[index] = nav2_costmap_2d::FREE_SPACE; // Out of bounds, mark as unknown
                }
            }
        }

        current_ = true;
    }

} // namespace foreground_mask

PLUGINLIB_EXPORT_CLASS(foreground_mask::ForegroundMaskLayer, nav2_costmap_2d::Layer)