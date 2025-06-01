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

        // Declare and get parameters
        declareParameter("publish_mask_only", rclcpp::ParameterValue(true));
        declareParameter("overwrite_costmap", rclcpp::ParameterValue(false));
        declareParameter("mask_cost_value", rclcpp::ParameterValue(255));
        declareParameter("map_topic", rclcpp::ParameterValue(std::string("/map")));

        std::string map_topic;
        node->get_parameter(name_ + "." + "publish_mask_only", publish_mask_only_);
        node->get_parameter(name_ + "." + "overwrite_costmap", overwrite_costmap_);
        node->get_parameter(name_ + "." + "mask_cost_value", mask_cost_);
        node->get_parameter(name_ + "." + "map_topic", map_topic);

        RCLCPP_INFO(node->get_logger(), "ForegroundMaskLayer: publish_mask_only=%s, overwrite_costmap=%s, mask_cost_value=%u, map_topic=%s",
                    publish_mask_only_ ? "true" : "false",
                    overwrite_costmap_ ? "true" : "false",
                    mask_cost_,
                    map_topic.c_str());

        // Subscribe to the static map
        map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&ForegroundMaskLayer::incomingStaticMap, this, std::placeholders::_1));

        // Initialize TF listener
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Publisher
        if (publish_mask_only_)
        {
            mask_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("foreground_mask", 1);
        }

        matchSize();
        current_ = true;
        enabled_ = true;
    }

    void ForegroundMaskLayer::incomingStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        static_map_ = msg;
        RCLCPP_INFO(logger_, "Static map received (%ux%u)", msg->info.width, msg->info.height);
    }

    void ForegroundMaskLayer::updateBounds(double, double, double,
                                           double *min_x, double *min_y,
                                           double *max_x, double *max_y)
    {
        if (!enabled_)
        {
            return; // If the layer is disabled, do nothing
        }

        // Expand the bounds to include the entire costmap
        *min_x = std::min(*min_x, layered_costmap_->getCostmap()->getOriginX());
        *min_y = std::min(*min_y, layered_costmap_->getCostmap()->getOriginY());
        *max_x = std::max(*max_x, layered_costmap_->getCostmap()->getOriginX() +
                                      layered_costmap_->getCostmap()->getSizeInMetersX());
        *max_y = std::max(*max_y, layered_costmap_->getCostmap()->getOriginY() +
                                      layered_costmap_->getCostmap()->getSizeInMetersY());
    }

    void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master,
                                          int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
    {

        // Overwrite the costmap with the foreground mask values
        // - mask_msg_.data: Stores the occupancy grid values (0 = free, 100 = occupied, -1 = unknown)
        // - master.setCost(): Updates the costmap with the corresponding cost values
        // Ensure compatibility with downstream processes like costmap_converter
        if (!static_map_)
        {
            RCLCPP_WARN(logger_, "Static map not available yet.");
            return; // Wait for the static map
        }

        // Prepare mask for publication
        if (publish_mask_only_)
        {
            mask_msg_.header.stamp = clock_->now();
            mask_msg_.header.frame_id = layered_costmap_->getGlobalFrameID();
            mask_msg_.info.width = master.getSizeInCellsX();
            mask_msg_.info.height = master.getSizeInCellsY();
            mask_msg_.info.resolution = master.getResolution();

            // Dynamically update the origin of the foreground mask to match the costmap's rolling window
            mask_msg_.info.origin.position.x = master.getOriginX();
            mask_msg_.info.origin.position.y = master.getOriginY();
            mask_msg_.info.origin.position.z = 0.0;
            mask_msg_.info.origin.orientation.w = 1.0;
            mask_msg_.info.origin.orientation.x = 0.0;
            mask_msg_.info.origin.orientation.y = 0.0;
            mask_msg_.info.origin.orientation.z = 0.0;

            mask_msg_.data.assign(mask_msg_.info.width * mask_msg_.info.height, 0);
        }

        // Transform static map to master costmap frame
        try
        {
            geometry_msgs::msg::TransformStamped tf_m2s = tf_buffer_->lookupTransform(
                layered_costmap_->getGlobalFrameID(), static_map_->header.frame_id, tf2::TimePointZero);

            for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y)
            {
                for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x)
                {
                    // Convert costmap cell to world coordinates
                    double wx, wy;
                    master.mapToWorld(x, y, wx, wy);

                    // Transform world coordinates to the static map frame
                    geometry_msgs::msg::PointStamped costmap_point, static_point;
                    costmap_point.header.frame_id = layered_costmap_->getGlobalFrameID();
                    costmap_point.point.x = wx;
                    costmap_point.point.y = wy;
                    costmap_point.point.z = 0.0;

                    tf2::doTransform(costmap_point, static_point, tf_m2s);

                    // Convert transformed world coordinates back to static map indices
                    unsigned int mx, my;
                    if (static_map_->info.resolution > 0 &&
                        static_point.point.x >= static_map_->info.origin.position.x &&
                        static_point.point.y >= static_map_->info.origin.position.y)
                    {
                        mx = static_cast<unsigned int>((static_point.point.x - static_map_->info.origin.position.x) /
                                                       static_map_->info.resolution);
                        my = static_cast<unsigned int>((static_point.point.y - static_map_->info.origin.position.y) /
                                                       static_map_->info.resolution);

                        if (mx < static_map_->info.width && my < static_map_->info.height)
                        {
                            unsigned int static_idx = mx + my * static_map_->info.width;
                            unsigned int costmap_idx = x + y * master.getSizeInCellsX();

                            int8_t master_val = master.getCost(x, y);
                            int8_t static_val = static_map_->data[static_idx];

                            // Debug logs for transformation and comparison
                            RCLCPP_DEBUG(logger_, "Costmap cell (%u, %u) -> World (%.2f, %.2f) -> Static map cell (%u, %u)",
                                         x, y, wx, wy, mx, my);

                            // Foreground obstacle detection logic
                            if (master_val < 0 && static_val <= 0) // Foreground obstacle
                            {
                                if (publish_mask_only_)
                                {
                                    mask_msg_.data[costmap_idx] = 100; // Set mask cost
                                }
                                // if (overwrite_costmap_)
                                // {
                                //     master.setCost(x, y, mask_cost_);
                                // }
                            }
                        }
                    }
                }
            }
        }
        catch (tf2::TransformException &ex)
        {
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        if (overwrite_costmap_ && publish_mask_only_)
        {
            mask_pub_->publish(mask_msg_);

            for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y)
            {
                for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x)
                {
                    unsigned int idx = x + y * master.getSizeInCellsX();
                    if (mask_msg_.data[idx] == 100) // Occupied cell in the foreground mask
                        master.setCost(x, y, static_cast<unsigned char>(mask_cost_)); // Set costmap value (e.g., 255 for LETHAL_OBSTACLE)
                    else if (mask_msg_.data[idx] == 0) // Free cell in the foreground mask
                        master.setCost(x, y, nav2_costmap_2d::FREE_SPACE); // Set costmap value to FREE_SPACE (0)
                    else // Unknown cell in the foreground mask
                        master.setCost(x, y, nav2_costmap_2d::NO_INFORMATION); // Set costmap value to NO_INFORMATION (-1)
                }
            }
        }

        else if (!overwrite_costmap_ && publish_mask_only_)
        {
            mask_pub_->publish(mask_msg_);
        }
    }

} // namespace foreground_mask

PLUGINLIB_EXPORT_CLASS(foreground_mask::ForegroundMaskLayer, nav2_costmap_2d::Layer)
