#include "foreground_mask_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace foreground_mask
{

    void ForegroundMaskLayer::onInitialize()
    {
        ObstacleLayer::onInitialize();
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

        matchSize();
        current_ = true;
        enabled_ = true;
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

    // void ForegroundMaskLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
    //                                        double *min_x, double *min_y,
    //                                        double *max_x, double *max_y)
    // {
    //     if (!enabled_ || !static_map_)
    //     {
    //         return;
    //     }

    //     std::lock_guard<Costmap2D::mutex_t> lock(*getMutex());

    //     if (layered_costmap_->isRolling())
    //     {
    //         updateOrigin(robot_x - getSizeInMetersX() / 2.0,
    //                      robot_y - getSizeInMetersY() / 2.0);
    //     }

    //     layer_min_x = getOriginX();
    //     layer_min_y = getOriginY();
    //     layer_max_x = layer_min_x + getSizeInMetersX();
    //     layer_max_y = layer_min_y + getSizeInMetersY();

    //     *min_x = std::min(*min_x, layer_min_x);
    //     *min_y = std::min(*min_y, layer_min_y);
    //     *max_x = std::max(*max_x, layer_max_x);
    //     *max_y = std::max(*max_y, layer_max_y);

    //     useExtraBounds(min_x, min_y, max_x, max_y);
    // }

    void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
                                          int min_i, int min_j, int max_i, int max_j)
    {
        ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);
        
        if (!enabled_ || !static_map_)
        {
            RCLCPP_WARN(logger_, "ForegroundMaskLayer is disabled or static map not available.");
            return;
        }

        for (int j = min_j; j < max_j; ++j)
        {
            for (int i = min_i; i < max_i; ++i)
            {
                unsigned int index = master_grid.getIndex(i, j);

                double wx, wy;
                master_grid.mapToWorld(i, j, wx, wy);

                unsigned int mx = static_cast<unsigned int>((wx - static_map_->info.origin.position.x) / static_map_->info.resolution);
                unsigned int my = static_cast<unsigned int>((wy - static_map_->info.origin.position.y) / static_map_->info.resolution);

                if (mx < static_map_->info.width && my < static_map_->info.height)
                {
                    unsigned int inflated_idx = mx + my * static_map_->info.width;

                    unsigned char inflated_val = inflated_static_[inflated_idx];
                    unsigned char master_val = master_grid.getCost(index);

                    // Comparison logic: foreground detection
                    if (master_val >= 90 && inflated_val < 50) // Detected as dynamic obstacle
                    {
                        master_grid.setCost(i, j, nav2_costmap_2d::LETHAL_OBSTACLE); // Mark as foreground obstacle
                    }
                    else
                    {
                        master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE); // Clear static/background data
                    }
                }
                else
                {
                    master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE); // Out of bounds, mark as unknown
                }
            }
        }

        current_ = true;
    }

    // void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master_grid,
    //                                       int min_i, int min_j, int max_i, int max_j)
    // {
    //     //------------------------------------------------------------------
    //     // 1.  Run the parent implementation.  This will:
    //     //     • copy the master_grid into our local costmap_
    //     //     • ray-trace clearing observations
    //     //     • mark lethal hits from the sensors
    //     //------------------------------------------------------------------
    //     ObstacleLayer::updateCosts(master_grid, min_i, min_j, max_i, max_j);

    //     //------------------------------------------------------------------
    //     // 2.  Our foreground logic: inspect every cell that MAY have changed
    //     //------------------------------------------------------------------
    //     if (!enabled_ || !static_map_)
    //     {
    //         return; // nothing to overwrite
    //     }

    //     for (int j = min_j; j < max_j; ++j)
    //     {
    //         for (int i = min_i; i < max_i; ++i)
    //         {
    //             //----------------------------------------------------------------
    //             // 2.1  We only care about cells that the parent marked lethal.
    //             //----------------------------------------------------------------
    //             unsigned char c = master_grid.getCost(i, j);
    //             if (c < nav2_costmap_2d::LETHAL_OBSTACLE)
    //                 continue; // leave FREE/INSCRATCH/NO_INFO alone

    //             //----------------------------------------------------------------
    //             // 2.2  Convert (i,j) to static-map indices
    //             //----------------------------------------------------------------
    //             double wx, wy;
    //             master_grid.mapToWorld(i, j, wx, wy);

    //             const auto &info = static_map_->info;
    //             int mx = static_cast<int>((wx - info.origin.position.x) / info.resolution);
    //             int my = static_cast<int>((wy - info.origin.position.y) / info.resolution);

    //             if (mx < 0 || my < 0 || mx >= static_cast<int>(info.width) ||
    //                 my >= static_cast<int>(info.height))
    //             {
    //                 continue; // outside static map – keep the mark
    //             }

    //             //----------------------------------------------------------------
    //             // 2.3  Compare with inflated static background
    //             //----------------------------------------------------------------
    //             unsigned int sidx = mx + my * info.width;
    //             unsigned char background = inflated_static_[sidx];

    //             // *** Your foreground criterion ***
    //             //   - keep the lethal mark only if the static map says "free"
    //             //   - otherwise restore whatever was there before (usually a low cost)
    //             if (background < 50)
    //             {                               // cell is free in static map
    //                 master_grid.setCost(i, j, nav2_costmap_2d::FREE_SPACE); // wipe dynamic artefact
    //             }
    //             // else: leave as LETHAL_OBSTACLE
    //         }
    //     }

    //     current_ = true;
    // }

} // namespace foreground_mask

PLUGINLIB_EXPORT_CLASS(foreground_mask::ForegroundMaskLayer, nav2_costmap_2d::Layer)