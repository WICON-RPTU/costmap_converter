#pragma once
#include "nav2_costmap_2d/layer.hpp"
#include "nav2_costmap_2d/layered_costmap.hpp"
#include "nav2_util/node_utils.hpp"
#include "tf2_ros/buffer.h"
#include "tf2_ros/transform_listener.h"
#include "nav_msgs/msg/occupancy_grid.hpp"
#include "geometry_msgs/msg/point_stamped.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp" // ❶ put BEFORE you call doTransform

namespace foreground_mask
{

    class ForegroundMaskLayer : public nav2_costmap_2d::Layer
    {
    public:
        ForegroundMaskLayer() = default;
        virtual ~ForegroundMaskLayer() = default;

        // ===== nav2 Layer interface =================================================
        void onInitialize() override;
        void updateBounds(double robot_x, double robot_y, double robot_yaw,
                          double *min_x, double *min_y,
                          double *max_x, double *max_y) override;
        void updateCosts(nav2_costmap_2d::Costmap2D &master,
                         int min_i, int min_j, int max_i, int max_j) override;
        void reset() override {}
        bool isClearable() override { return false; }
        

    private:
        // ---------- parameters ------------------------------------------------------
        std::string map_topic_;              // topic to subscribe to for static map

        // ---------- static map copy -------------------------------------------------
        nav_msgs::msg::OccupancyGrid::SharedPtr static_map_;
        rclcpp::Subscription<nav_msgs::msg::OccupancyGrid>::SharedPtr map_sub_;

        std::vector<int8_t> inflated_static_; // Inflated static map buffer
        int inflation_radius_;


        // ---------- publisher -------------------------------------------------------
        rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr mask_pub_;
        nav_msgs::msg::OccupancyGrid mask_msg_;

        // ---------- helpers ---------------------------------------------------------
        void incomingStaticMap(const nav_msgs::msg::OccupancyGrid::SharedPtr msg);

    };

} // namespace foreground_mask
