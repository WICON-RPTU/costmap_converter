#include "foreground_mask_layer.hpp"
#include "pluginlib/class_list_macros.hpp"

namespace foreground_mask
{

    // Called when the layer is initialized (plugin startup)
    // It declares and retrieves parameters, subscribes to the global static map,
    // initializes the TF listener for transforms, and creates a publisher if needed.
    void ForegroundMaskLayer::onInitialize()
    {
        auto node = node_.lock();
        if (!node)
        {
            throw std::runtime_error{"Failed to lock node. Ensure node_ is initialized properly."};
        }

        // Declare plugin parameters.
        // These parameters come from the YAML (like inflation_radius, map_topic, etc.).
        declareParameter("publish_mask_only", rclcpp::ParameterValue(true));
        declareParameter("overwrite_costmap", rclcpp::ParameterValue(false));
        declareParameter("mask_cost_value", rclcpp::ParameterValue(255));
        declareParameter("map_topic", rclcpp::ParameterValue(std::string("/map")));
        declareParameter("inflation_radius", rclcpp::ParameterValue(8)); // Default inflation radius

        std::string map_topic;
        node->get_parameter(name_ + "." + "publish_mask_only", publish_mask_only_);
        node->get_parameter(name_ + "." + "overwrite_costmap", overwrite_costmap_);
        node->get_parameter(name_ + "." + "mask_cost_value", mask_cost_);
        node->get_parameter(name_ + "." + "map_topic", map_topic);
        node->get_parameter(name_ + "." + "inflation_radius", inflation_radius_); // Get the inflation radius

        RCLCPP_INFO(node->get_logger(), "ForegroundMaskLayer: publish_mask_only=%s, overwrite_costmap=%s, mask_cost_value=%u, map_topic=%s, inflation_radius=%d",
                    publish_mask_only_ ? "true" : "false",
                    overwrite_costmap_ ? "true" : "false",
                    mask_cost_,
                    map_topic.c_str(),
                    inflation_radius_);

        // Subscribe to the static map on the provided topic.
        map_sub_ = node->create_subscription<nav_msgs::msg::OccupancyGrid>(
            map_topic,
            rclcpp::QoS(1).transient_local().reliable(),
            std::bind(&ForegroundMaskLayer::incomingStaticMap, this, std::placeholders::_1));

        // Set up TF listener so we can transform points between frames.
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(node->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

        // Create a publisher to output the foreground mask if needed.
        if (publish_mask_only_)
        {
            mask_pub_ = node->create_publisher<nav_msgs::msg::OccupancyGrid>("foreground_mask", 1);
        }

        matchSize();
        current_ = true;
        enabled_ = true;
    }


    // incomingStaticMap is called when a new global static map message arrives.
    // It saves that map and then builds an "inflated" version in the 
    // inflated_static_ buffer. The inflation adds a margin around obstacles.
    void ForegroundMaskLayer::incomingStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg)
    {
        static_map_ = msg;
        RCLCPP_INFO(logger_, "Static map received (%ux%u)", msg->info.width, msg->info.height);

        // Resize the inflated static map buffer to hold one cell per original map cell.
        inflated_static_.resize(static_map_->info.width * static_map_->info.height, 0);

        // Inflate the static map using the parameter inflation_radius_.
        // For each cell in the static map:
        //   If the cell is occupied (value == 100),
        //   mark all cells within the specified radius as "inflated" (set value 90)
        for (unsigned int y = 0; y < static_map_->info.height; ++y)
        {
            for (unsigned int x = 0; x < static_map_->info.width; ++x)
            {
                unsigned int idx = x + y * static_map_->info.width;
                if (static_map_->data[idx] == 100) // Obstacle cell in static map
                {
                    // Iterate over a square centered at (x,y) with a radius of inflation_radius_
                    for (int dy = -inflation_radius_; dy <= inflation_radius_; ++dy)
                    {
                        for (int dx = -inflation_radius_; dx <= inflation_radius_; ++dx)
                        {
                            int nx = x + dx;
                            int ny = y + dy;
                            // Check bounds with casting to int to avoid signed/unsigned issues
                            if (nx >= 0 && ny >= 0 &&
                                nx < static_cast<int>(static_map_->info.width) &&
                                ny < static_cast<int>(static_map_->info.height))
                            {
                                unsigned int inflated_idx = nx + ny * static_map_->info.width;
                                inflated_static_[inflated_idx] = 90; // Mark as inflated obstacle value
                            }
                        }
                    }
                }
            }
        }
        RCLCPP_INFO(logger_, "Static map inflated.");
    }


    // updateBounds is called to update the region of the costmap that may change.
    // If using a rolling window, the origin of the costmap is updated based on the robot position.
    void ForegroundMaskLayer::updateBounds(double robot_x, double robot_y, double /*robot_yaw*/,
                                           double *min_x, double *min_y,
                                           double *max_x, double *max_y)
    {
        if (!enabled_)
            return;

        // If using a rolling window, re-center the costmap around the robot.
        if (layered_costmap_->isRolling())
        {
            layered_costmap_->getCostmap()->updateOrigin(
                robot_x - layered_costmap_->getCostmap()->getSizeInMetersX() / 2,
                robot_y - layered_costmap_->getCostmap()->getSizeInMetersY() / 2);
        }

        // Expand the updating bounds to cover the entire costmap.
        *min_x = std::min(*min_x, layered_costmap_->getCostmap()->getOriginX());
        *min_y = std::min(*min_y, layered_costmap_->getCostmap()->getOriginY());
        *max_x = std::max(*max_x, layered_costmap_->getCostmap()->getOriginX() +
                                      layered_costmap_->getCostmap()->getSizeInMetersX());
        *max_y = std::max(*max_y, layered_costmap_->getCostmap()->getOriginY() +
                                      layered_costmap_->getCostmap()->getSizeInMetersY());
    }


    // updateCosts performs the main job: it compares the live costmap against the inflated static map.
    // It uses a TF transformation to bring costmap cells into the map frame for proper alignment.
    // Cells in the costmap that show an obstacle (≥90) but where the inflated map does not (inflated value < 50)
    // are marked as dynamic/foreground obstacles.
    void ForegroundMaskLayer::updateCosts(nav2_costmap_2d::Costmap2D &master,
                                          int /*min_i*/, int /*min_j*/, int /*max_i*/, int /*max_j*/)
    {
        if (!static_map_)
        {
            RCLCPP_WARN(logger_, "Static map not available yet.");
            return; // Wait for the static map
        }

        // If publishing mask is enabled, initialize the mask message fields based on the costmap.
        if (publish_mask_only_)
        {
            mask_msg_.header.stamp = clock_->now();
            mask_msg_.header.frame_id = layered_costmap_->getGlobalFrameID();
            mask_msg_.info.width = master.getSizeInCellsX();
            mask_msg_.info.height = master.getSizeInCellsY();
            mask_msg_.info.resolution = master.getResolution();

            mask_msg_.info.origin.position.x = master.getOriginX();
            mask_msg_.info.origin.position.y = master.getOriginY();
            mask_msg_.info.origin.position.z = 0.0;
            mask_msg_.info.origin.orientation.w = 1.0;
            mask_msg_.info.origin.orientation.x = 0.0;
            mask_msg_.info.origin.orientation.y = 0.0;
            mask_msg_.info.origin.orientation.z = 0.0;

            mask_msg_.data.assign(mask_msg_.info.width * mask_msg_.info.height, 0);
        }

        // Look up TF transform to convert costmap (usually in odom) into the map frame.
        // This compensates for drift or offset.
        try
        {
            geometry_msgs::msg::TransformStamped tf_map_to_costmap = tf_buffer_->lookupTransform(
                "map", layered_costmap_->getGlobalFrameID(), tf2::TimePointZero);

            tf2::Quaternion q(
                tf_map_to_costmap.transform.rotation.x,
                tf_map_to_costmap.transform.rotation.y,
                tf_map_to_costmap.transform.rotation.z,
                tf_map_to_costmap.transform.rotation.w);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

            double cos_yaw = std::cos(yaw);
            double sin_yaw = std::sin(yaw);

            // Iterate over every cell in the master costmap.
            for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y)
            {
                for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x)
                {
                    double wx, wy;
                    // Convert the costmap cell (x,y) into world coordinates.
                    master.mapToWorld(x, y, wx, wy);

                    // Transform world coordinates to the map frame:
                    double dx = wx - tf_map_to_costmap.transform.translation.x;
                    double dy = wy - tf_map_to_costmap.transform.translation.y;
                    double wx_map = cos_yaw * dx - sin_yaw * dy;
                    double wy_map = sin_yaw * dx + cos_yaw * dy;

                    unsigned int mx, my;
                    // Convert the transformed map coordinates into indices
                    if (static_map_->info.resolution > 0 &&
                        wx_map >= static_map_->info.origin.position.x &&
                        wy_map >= static_map_->info.origin.position.y)
                    {
                        mx = static_cast<unsigned int>((wx_map - static_map_->info.origin.position.x) /
                                                       static_map_->info.resolution);
                        my = static_cast<unsigned int>((wy_map - static_map_->info.origin.position.y) /
                                                       static_map_->info.resolution);

                        // Check that the indices are within the boundaries of the static map.
                        if (mx < static_map_->info.width && my < static_map_->info.height)
                        {
                            unsigned int inflated_idx = mx + my * static_map_->info.width;
                            unsigned int costmap_idx = x + y * master.getSizeInCellsX();

                            // Get the live cost from the master costmap and the inflated static value.
                            double master_val = master.getCost(x, y);
                            int8_t inflated_val = inflated_static_[inflated_idx];

                            // If the costmap indicates an obstacle (value ≥ 90) but the inflated static map does not (value < 50),
                            // mark this cell in the foreground mask and optionally update the master costmap.
                            if (master_val >= 90 && inflated_val < 50)
                            {
                                if (publish_mask_only_)
                                {
                                    mask_msg_.data[costmap_idx] = 100;
                                }
                                if (overwrite_costmap_)
                                {
                                    master.setCost(x, y, mask_cost_);
                                }
                            }
                        }
                    }
                }
            }
        }
        catch (tf2::TransformException &ex)
        {
            // If TF lookup fails, log the warning and exit.
            RCLCPP_WARN_THROTTLE(logger_, *clock_, 2000, "TF lookup failed: %s", ex.what());
            return;
        }

        // Depending on configuration, either publish the mask and/or overwrite the master costmap.
        if (overwrite_costmap_ && publish_mask_only_)
        {
            mask_pub_->publish(mask_msg_);

            for (unsigned int y = 0; y < master.getSizeInCellsY(); ++y)
            {
                for (unsigned int x = 0; x < master.getSizeInCellsX(); ++x)
                {
                    unsigned int idx = x + y * master.getSizeInCellsX();
                    if (mask_msg_.data[idx] == 100)                                   // Occupied in mask
                        master.setCost(x, y, static_cast<unsigned char>(mask_cost_));
                    else if (mask_msg_.data[idx] == 0)                                // Free cell
                        master.setCost(x, y, nav2_costmap_2d::FREE_SPACE);
                    else                                                              // Otherwise, no information
                        master.setCost(x, y, nav2_costmap_2d::NO_INFORMATION);
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
