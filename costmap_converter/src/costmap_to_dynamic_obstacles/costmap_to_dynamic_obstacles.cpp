#include <costmap_converter/costmap_to_dynamic_obstacles/costmap_to_dynamic_obstacles.h>

#include <pluginlib/class_list_macros.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

PLUGINLIB_EXPORT_CLASS(costmap_converter::CostmapToDynamicObstacles, costmap_converter::BaseCostmapToPolygons)

namespace costmap_converter
{

  CostmapToDynamicObstacles::CostmapToDynamicObstacles() : BaseCostmapToDynamicObstacles()
  {
    ego_vel_.x = ego_vel_.y = ego_vel_.z = 0;
    costmap_ = nullptr;
  }

  CostmapToDynamicObstacles::~CostmapToDynamicObstacles()
  {
  }

  void CostmapToDynamicObstacles::initialize(rclcpp::Node::SharedPtr nh)
  {
    BaseCostmapToPolygons::initialize(nh);

    costmap_ = nullptr;

    // We need the odometry from the robot to compensate the ego motion
    //  rclcpp::Node::SharedPtr gn; // create odom topic w.r.t. global node handle
    odom_sub_ = nh->create_subscription<nav_msgs::msg::Odometry>(
        odom_topic_,
        rclcpp::SystemDefaultsQoS(),
        std::bind(&CostmapToDynamicObstacles::odomCallback, this, std::placeholders::_1));
    nh->declare_parameter("dynamic_obstacle_plugin.publish_static_obstacles", rclcpp::ParameterValue(true));
    nh->get_parameter("dynamic_obstacle_plugin.publish_static_obstacles", publish_static_obstacles_);

    //////////////////////////////////
    // Foreground detection parameters
    BackgroundSubtractor::Params bg_sub_params;

    nh->declare_parameter("bg_sub_params.alpha_slow", rclcpp::ParameterValue(0.3));
    nh->get_parameter("bg_sub_params.alpha_slow", bg_sub_params.alpha_slow);

    nh->declare_parameter("bg_sub_params.alpha_fast", rclcpp::ParameterValue(0.9));
    nh->get_parameter("bg_sub_params.alpha_fast", bg_sub_params.alpha_fast);

    nh->declare_parameter("bg_sub_params.beta", rclcpp::ParameterValue(0.9));
    nh->get_parameter("bg_sub_params.beta", bg_sub_params.beta);

    nh->declare_parameter("bg_sub_params.min_occupancy_probability", rclcpp::ParameterValue(200.0));
    nh->get_parameter("bg_sub_params.min_occupancy_probability", bg_sub_params.min_occupancy_probability);

    nh->declare_parameter("bg_sub_params.min_sep_between_fast_and_slow_filter", rclcpp::ParameterValue(80.0));
    nh->get_parameter("bg_sub_params.min_sep_between_fast_and_slow_filter", bg_sub_params.min_sep_between_fast_and_slow_filter);

    nh->declare_parameter("bg_sub_params.max_occupancy_neighbors", rclcpp::ParameterValue(100.0));
    nh->get_parameter("bg_sub_params.max_occupancy_neighbors", bg_sub_params.max_occupancy_neighbors);

    nh->declare_parameter("bg_sub_params.morph_size", rclcpp::ParameterValue(1));
    nh->get_parameter("bg_sub_params.morph_size", bg_sub_params.morph_size);

    bg_sub_ = std::unique_ptr<BackgroundSubtractor>(new BackgroundSubtractor(bg_sub_params));

    ////////////////////////////
    // Blob detection parameters
    BlobDetector::Params blob_det_params;

    nh->declare_parameter("blob_det_params.filter_by_color", rclcpp::ParameterValue(true)); // Always true, actually filterByIntensity
    nh->get_parameter("blob_det_params.filter_by_color", blob_det_params.filterByColor);

    nh->declare_parameter("blob_det_params.blob_color", rclcpp::ParameterValue(255)); // Extract light blobs
    nh->get_parameter("blob_det_params.blob_color", blob_det_params.blobColor);

    nh->declare_parameter("blob_det_params.threshold_step", rclcpp::ParameterValue(256.0)); // Input for blob detection is already a binary image
    nh->get_parameter("blob_det_params.threshold_step", blob_det_params.thresholdStep);

    nh->declare_parameter("blob_det_params.min_threshold", rclcpp::ParameterValue(127.0));
    nh->get_parameter("blob_det_params.min_threshold", blob_det_params.minThreshold);

    nh->declare_parameter("blob_det_params.max_threshold", rclcpp::ParameterValue(255.0));
    nh->get_parameter("blob_det_params.max_threshold", blob_det_params.maxThreshold);

    nh->declare_parameter("blob_det_params.min_repeatability", rclcpp::ParameterValue(1));
    nh->get_parameter("blob_det_params.min_repeatability", blob_det_params.minRepeatability);

    nh->declare_parameter("blob_det_params.min_distance_between_blobs", rclcpp::ParameterValue(10.0));
    nh->get_parameter("blob_det_params.min_distance_between_blobs", blob_det_params.minDistBetweenBlobs);

    nh->declare_parameter("blob_det_params.filter_by_area", rclcpp::ParameterValue(true));
    nh->get_parameter("blob_det_params.filter_by_area", blob_det_params.filterByArea);

    nh->declare_parameter("blob_det_params.min_area", rclcpp::ParameterValue(3.0)); // Filter out blobs with fewer pixels
    nh->get_parameter("blob_det_params.min_area", blob_det_params.minArea);

    nh->declare_parameter("blob_det_params.max_area", rclcpp::ParameterValue(300.0));
    nh->get_parameter("blob_det_params.max_area", blob_det_params.maxArea);

    nh->declare_parameter("blob_det_params.filter_by_circularity", rclcpp::ParameterValue(true)); // circularity = 4*pi*area/perimeter^2
    nh->get_parameter("blob_det_params.filter_by_circularity", blob_det_params.filterByCircularity);

    nh->declare_parameter("blob_det_params.min_circularity", rclcpp::ParameterValue(0.2));
    nh->get_parameter("blob_det_params.min_circularity", blob_det_params.minCircularity);

    nh->declare_parameter("blob_det_params.max_circularity", rclcpp::ParameterValue(1.0)); // Maximal 1 (in case of a circle)
    nh->get_parameter("blob_det_params.max_circularity", blob_det_params.maxCircularity);

    nh->declare_parameter("blob_det_params.filter_by_inertia", rclcpp::ParameterValue(true)); // Filter blobs based on their elongation
    nh->get_parameter("blob_det_params.filter_by_inertia", blob_det_params.filterByInertia);

    nh->declare_parameter("blob_det_params.min_inertia_ratio", rclcpp::ParameterValue(0.2)); // Minimal 0 (in case of a line)
    nh->get_parameter("blob_det_params.min_inertia_ratio", blob_det_params.minInertiaRatio);

    nh->declare_parameter("blob_det_params.max_inertia_ratio", rclcpp::ParameterValue(1.0)); // Maximal 1 (in case of a circle)
    nh->get_parameter("blob_det_params.max_inertia_ratio", blob_det_params.maxInertiaRatio);

    nh->declare_parameter("blob_det_params.filter_by_convexity", rclcpp::ParameterValue(false)); // Area of the Blob / Area of its convex hull
    nh->get_parameter("blob_det_params.filter_by_convexity", blob_det_params.filterByConvexity);

    nh->declare_parameter("blob_det_params.min_convexity", rclcpp::ParameterValue(0.0)); // Minimal 0
    nh->get_parameter("blob_det_params.min_convexity", blob_det_params.minConvexity);

    nh->declare_parameter("blob_det_params.max_convexity", rclcpp::ParameterValue(1.0)); // Maximal 1
    nh->get_parameter("blob_det_params.max_convexity", blob_det_params.maxConvexity);

    blob_det_ = BlobDetector::create(blob_det_params);

    ////////////////////////////////////
    // Tracking parameters
    CTracker::Params tracker_params;

    nh->declare_parameter("tracker_params.dt", rclcpp::ParameterValue(0.1));
    nh->get_parameter("tracker_params.dt", tracker_params.dt);

    nh->declare_parameter("tracker_params.dist_thresh", rclcpp::ParameterValue(15.0));
    nh->get_parameter("tracker_params.dist_thresh", tracker_params.dist_thresh);

    nh->declare_parameter("tracker_params.max_allowed_skipped_frames", rclcpp::ParameterValue(3));
    nh->get_parameter("tracker_params.max_allowed_skipped_frames", tracker_params.max_allowed_skipped_frames);

    nh->declare_parameter("tracker_params.max_trace_length", rclcpp::ParameterValue(10));
    nh->get_parameter("tracker_params.max_trace_length", tracker_params.max_trace_length);

    tracker_ = std::unique_ptr<CTracker>(new CTracker(tracker_params));

    ////////////////////////////////////
    // Static costmap conversion parameters
    std::string static_converter_plugin;
    nh->declare_parameter("dynamic_obstacle_plugin.static_converter_plugin", rclcpp::ParameterValue("costmap_converter::CostmapToPolygonsDBSMCCH"));
    nh->get_parameter("dynamic_obstacle_plugin.static_converter_plugin", static_converter_plugin);
    loadStaticCostmapConverterPlugin(static_converter_plugin, nh);

    // setup dynamic reconfigure
    callback_handle = nh->add_on_set_parameters_callback(std::bind(&CostmapToDynamicObstacles::parameters_callback, this, std::placeholders::_1));
  }


  rcl_interfaces::msg::SetParametersResult CostmapToDynamicObstacles::parameters_callback(const std::vector<rclcpp::Parameter> &parameters)
  {
    ////////////////////////////
    // Dynamic parameter reconfiguration 
    rcl_interfaces::msg::SetParametersResult result;
    result.successful = true;
    result.reason = "";
    // Flag to decide if `updateParameters` should be called
    bool update_bg_sub = false;
    bool update_blob_det = false;
    bool update_tracker = false;
    // Set class attributes
    for (const rclcpp::Parameter &parameter : parameters)
    {
      if (parameter.get_name() == "dynamic_obstacle_plugin.static_converter_plugin")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }
      else if (parameter.get_name() == "dynamic_obstacle_plugin.publish_static_obstacles")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }

      // Static Layer Parameter
      else if (parameter.get_name() == "PolygonDBSMCCH.cluster_max_distance")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }
      else if (parameter.get_name() == "PolygonDBSMCCH.cluster_max_pts")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }
      else if (parameter.get_name() == "PolygonDBSMCCH.cluster_min_pts")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }
      else if (parameter.get_name() == "PolygonDBSMCCH.convex_hull_min_pt_separation")
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change had not effect, as it is not advisable to change it at runtime.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }

      // Background Subtractor Parameters
      else if (parameter.get_name() == "bg_sub_params.alpha_slow")
      {
        this->bg_sub_->bg_sub_params.alpha_slow = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.alpha_fast")
      {
        this->bg_sub_->bg_sub_params.alpha_fast = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.beta")
      {
        this->bg_sub_->bg_sub_params.beta = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.min_sep_between_fast_and_slow_filter")
      {
        this->bg_sub_->bg_sub_params.min_sep_between_fast_and_slow_filter = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.min_occupancy_probability")
      {
        this->bg_sub_->bg_sub_params.min_occupancy_probability = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.max_occupancy_neighbors")
      {
        this->bg_sub_->bg_sub_params.max_occupancy_neighbors = parameter.as_double();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "bg_sub_params.morph_size")
      {
        this->bg_sub_->bg_sub_params.morph_size = parameter.as_int();
        if (!update_bg_sub)
        {
          update_bg_sub = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_int());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }

      // Blob Detector Parameters
      else if (parameter.get_name() == "blob_det_params.filter_by_color")
      {
        this->blob_det_->blob_det_params.filterByColor = parameter.as_bool();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_bool());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.blob_color")
      {
        this->blob_det_->blob_det_params.blobColor = parameter.as_int();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_int());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.threshold_step")
      {
        this->blob_det_->blob_det_params.thresholdStep = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_threshold")
      {
        this->blob_det_->blob_det_params.minThreshold = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.max_threshold")
      {
        this->blob_det_->blob_det_params.maxThreshold = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_repeatability")
      {
        this->blob_det_->blob_det_params.minRepeatability = parameter.as_int();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_int());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_distance_between_blobs")
      {
        this->blob_det_->blob_det_params.minDistBetweenBlobs = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.filter_by_area")
      {
        this->blob_det_->blob_det_params.filterByArea = parameter.as_bool();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_bool());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_area")
      {
        this->blob_det_->blob_det_params.minArea = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.max_area")
      {
        this->blob_det_->blob_det_params.maxArea = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.filter_by_circularity")
      {
        this->blob_det_->blob_det_params.filterByCircularity = parameter.as_bool();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_bool());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_circularity")
      {
        this->blob_det_->blob_det_params.minCircularity = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.max_circularity")
      {
        this->blob_det_->blob_det_params.maxCircularity = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.filter_by_convexity")
      {
        this->blob_det_->blob_det_params.filterByConvexity = parameter.as_bool();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_bool());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_convexity")
      {
        this->blob_det_->blob_det_params.minConvexity = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.max_convexity")
      {
        this->blob_det_->blob_det_params.maxConvexity = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.filter_by_inertia")
      {
        this->blob_det_->blob_det_params.filterByInertia = parameter.as_bool();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_bool());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.min_inertia_ratio")
      {
        this->blob_det_->blob_det_params.minInertiaRatio = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "blob_det_params.max_inertia_ratio")
      {
        this->blob_det_->blob_det_params.maxInertiaRatio = parameter.as_double();
        if (!update_blob_det)
        {
          update_blob_det = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }

      // Tracker Parameters
      else if (parameter.get_name() == "tracker_params.dt")
      {
        this->tracker_->tracker_params.dt = parameter.as_double();
        if (!update_tracker)
        {
          update_tracker = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "tracker_params.dist_thresh")
      {
        this->tracker_->tracker_params.dist_thresh = parameter.as_double();
        if (!update_tracker)
        {
          update_tracker = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_double());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "tracker_params.max_allowed_skipped_frames")
      {
        this->tracker_->tracker_params.max_allowed_skipped_frames = parameter.as_int();
        if (!update_tracker)
        {
          update_tracker = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_int());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }
      else if (parameter.get_name() == "tracker_params.max_trace_length")
      {
        this->tracker_->tracker_params.max_trace_length = parameter.as_int();
        if (!update_tracker)
        {
          update_tracker = true;
        }
        std::string success_message = "Parameter " + parameter.get_name() + " was changed to " + std::to_string(parameter.as_int());
        RCLCPP_INFO(this->getLogger(), success_message.c_str());
        result.reason = result.reason + std::string("\n") + success_message;
      }

      else
      {
        result.successful = false;
        std::string failure_message = "Parameter " + parameter.get_name() + " change was not successful. It is either not implemented or the parameter does not exist.";
        RCLCPP_WARN(this->getLogger(), failure_message.c_str());
        result.reason = result.reason + std::string("\n") + failure_message;
      }
    }
    // Update BackgroundSubtractor if parameters changed
    if (update_bg_sub)
    {
      BackgroundSubtractor::Params bg_params;
      bg_params.alpha_slow = this->bg_sub_->bg_sub_params.alpha_slow;
      bg_params.alpha_fast = this->bg_sub_->bg_sub_params.alpha_fast;
      bg_params.beta = this->bg_sub_->bg_sub_params.beta;
      bg_params.min_sep_between_fast_and_slow_filter = this->bg_sub_->bg_sub_params.min_sep_between_fast_and_slow_filter;
      bg_params.min_occupancy_probability = this->bg_sub_->bg_sub_params.min_occupancy_probability;
      bg_params.max_occupancy_neighbors = this->bg_sub_->bg_sub_params.max_occupancy_neighbors;
      bg_params.morph_size = this->bg_sub_->bg_sub_params.morph_size;
      this->bg_sub_->updateParameters(bg_params);
      RCLCPP_WARN(this->getLogger(), "BackgroundSubtractor parameters updated.");
    }

    // Update BlobDetector if parameters changed
    if (update_blob_det)
    {
      BlobDetector::Params blob_params;
      blob_params.filterByColor = this->blob_det_->blob_det_params.filterByColor;
      blob_params.blobColor = this->blob_det_->blob_det_params.blobColor;
      blob_params.thresholdStep = this->blob_det_->blob_det_params.thresholdStep;
      blob_params.minThreshold = this->blob_det_->blob_det_params.minThreshold;
      blob_params.maxThreshold = this->blob_det_->blob_det_params.maxThreshold;
      blob_params.minRepeatability = this->blob_det_->blob_det_params.minRepeatability;
      blob_params.minDistBetweenBlobs = this->blob_det_->blob_det_params.minDistBetweenBlobs;
      blob_params.filterByArea = this->blob_det_->blob_det_params.filterByArea;
      blob_params.minArea = this->blob_det_->blob_det_params.minArea;
      blob_params.maxArea = this->blob_det_->blob_det_params.maxArea;
      blob_params.filterByCircularity = this->blob_det_->blob_det_params.filterByCircularity;
      blob_params.minCircularity = this->blob_det_->blob_det_params.minCircularity;
      blob_params.maxCircularity = this->blob_det_->blob_det_params.maxCircularity;
      blob_params.filterByConvexity = this->blob_det_->blob_det_params.filterByConvexity;
      blob_params.minConvexity = this->blob_det_->blob_det_params.minConvexity;
      blob_params.maxConvexity = this->blob_det_->blob_det_params.maxConvexity;
      blob_params.filterByInertia = this->blob_det_->blob_det_params.filterByInertia;
      blob_params.minInertiaRatio = this->blob_det_->blob_det_params.minInertiaRatio;
      blob_params.maxInertiaRatio = this->blob_det_->blob_det_params.maxInertiaRatio;
      this->blob_det_->updateParameters(blob_params);
      RCLCPP_WARN(this->getLogger(), "BlobDetector parameters updated.");
    }

    // Update Tracker if parameters changed
    if (update_tracker)
    {
      CTracker::Params tracker_params_internal;
      tracker_params_internal.dt = this->tracker_->tracker_params.dt;
      tracker_params_internal.dist_thresh = this->tracker_->tracker_params.dist_thresh;
      tracker_params_internal.max_allowed_skipped_frames = this->tracker_->tracker_params.max_allowed_skipped_frames;
      tracker_params_internal.max_trace_length = this->tracker_->tracker_params.max_trace_length;
      this->tracker_->updateParameters(tracker_params_internal);
      RCLCPP_WARN(this->getLogger(), "Tracker parameters updated.");
    }

    return result;
  }

  void CostmapToDynamicObstacles::compute()
  {
    if (costmap_mat_.empty())
      return;

    /////////////////////////// Foreground detection ////////////////////////////////////
    // Dynamic obstacles are separated from static obstacles
    int origin_x = round(costmap_->getOriginX() / costmap_->getResolution());
    int origin_y = round(costmap_->getOriginY() / costmap_->getResolution());
    // ROS_INFO("Origin x  [m]: %f    Origin_y  [m]: %f", costmap_->getOriginX(), costmap_->getOriginY());
    // ROS_INFO("Origin x [px]: %d \t Origin_y [px]: %d", originX, originY);

    bg_sub_->apply(costmap_mat_, fg_mask_, origin_x, origin_y);

    // if no foreground object is detected, no ObstacleMsgs need to be published
    if (fg_mask_.empty())
      return;

    cv::Mat bg_mat;
    if (publish_static_obstacles_)
    {
      // Get static obstacles
      bg_mat = costmap_mat_ - fg_mask_;
      // visualize("bg_mat", bg_mat);
    }

    /////////////////////////////// Blob detection /////////////////////////////////////
    // Centers and contours of Blobs are detected
    blob_det_->detect(fg_mask_, keypoints_);
    std::vector<std::vector<cv::Point>> contours = blob_det_->getContours();

    ////////////////////////////// Tracking ////////////////////////////////////////////
    // Objects are assigned to objects from previous frame based on Hungarian Algorithm
    // Object velocities are estimated using a Kalman Filter
    std::vector<Point_t> detected_centers(keypoints_.size());
    for (size_t i = 0; i < keypoints_.size(); i++)
    {
      detected_centers.at(i).x = keypoints_.at(i).pt.x;
      detected_centers.at(i).y = keypoints_.at(i).pt.y;
      detected_centers.at(i).z = 0; // Currently unused!
    }

    tracker_->Update(detected_centers, contours);

    ///////////////////////////////////// Output ///////////////////////////////////////
    /*
    cv::Mat fg_mask_with_keypoints = cv::Mat::zeros(fg_mask.size(), CV_8UC3);
    cv::cvtColor(fg_mask, fg_mask_with_keypoints, cv::COLOR_GRAY2BGR);

    for (int i = 0; i < (int)tracker_->tracks.size(); ++i)
      cv::rectangle(fg_mask_with_keypoints, cv::boundingRect(tracker_->tracks[i]->getLastContour()),
                    cv::Scalar(0, 0, 255), 1);

    //visualize("fgMaskWithKeyPoints", fgMaskWithKeypoints);
    //cv::imwrite("/home/albers/Desktop/fgMask.png", fgMask);
    //cv::imwrite("/home/albers/Desktop/fgMaskWithKeyPoints.png", fgMaskWithKeypoints);
    */

    //////////////////////////// Fill ObstacleContainerPtr /////////////////////////////
    ObstacleArrayPtr obstacles(new costmap_converter_msgs::msg::ObstacleArrayMsg);
    // header.seq is automatically filled
    obstacles->header.stamp = now();
    obstacles->header.frame_id = "/map"; // Global frame /map

    // For all tracked objects
    for (unsigned int i = 0; i < (unsigned int)tracker_->tracks.size(); ++i)
    {
      geometry_msgs::msg::Polygon polygon;

      // TODO directly create polygon inside getContour and avoid copy
      std::vector<Point_t> contour;
      getContour(i, contour); // this method also transforms map to world coordinates

      // convert contour to polygon
      for (const Point_t &pt : contour)
      {
        polygon.points.emplace_back();
        polygon.points.back().x = pt.x;
        polygon.points.back().y = pt.y;
        polygon.points.back().z = 0;
      }

      obstacles->obstacles.emplace_back();
      obstacles->obstacles.back().polygon = polygon;

      // Set obstacle ID
      obstacles->obstacles.back().id = tracker_->tracks.at(i)->track_id;

      // Set orientation
      geometry_msgs::msg::QuaternionStamped orientation;

      Point_t vel = getEstimatedVelocityOfObject(i);
      double yaw = std::atan2(vel.y, vel.x);
      // ROS_INFO("yaw: %f", yaw);
      tf2::Quaternion q;
      q.setRPY(0, 0, yaw);
      obstacles->obstacles.back().orientation = tf2::toMsg(q);

      // Set velocity
      geometry_msgs::msg::TwistWithCovariance velocities;
      // velocities.twist.linear.x = std::sqrt(vel.x*vel.x + vel.y*vel.y);
      // velocities.twist.linear.y = 0;
      velocities.twist.linear.x = vel.x;
      velocities.twist.linear.y = vel.y; // TODO(roesmann): don't we need to consider the transformation between opencv's and costmap's coordinate frames?
      velocities.twist.linear.z = 0;
      velocities.twist.angular.x = 0;
      velocities.twist.angular.y = 0;
      velocities.twist.angular.z = 0;

      // TODO: use correct covariance matrix
      velocities.covariance = {1, 0, 0, 0, 0, 0,
                               0, 1, 0, 0, 0, 0,
                               0, 0, 1, 0, 0, 0,
                               0, 0, 0, 1, 0, 0,
                               0, 0, 0, 0, 1, 0,
                               0, 0, 0, 0, 0, 1};

      obstacles->obstacles.back().velocities = velocities;
    }

    ////////////////////////// Static obstacles ////////////////////////////
    if (publish_static_obstacles_)
    {
      uchar *img_data = bg_mat.data;
      int width = bg_mat.cols;
      int height = bg_mat.rows;
      int stride = bg_mat.step;

      if (stackedCostmapConversion())
      {
        // Create new costmap with static obstacles (background)
        std::shared_ptr<nav2_costmap_2d::Costmap2D> static_costmap(new nav2_costmap_2d::Costmap2D(costmap_->getSizeInCellsX(),
                                                                                                  costmap_->getSizeInCellsY(),
                                                                                                  costmap_->getResolution(),
                                                                                                  costmap_->getOriginX(),
                                                                                                  costmap_->getOriginY()));
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            static_costmap->setCost(j, i, img_data[i * stride + j]);
          }
        }

        // Apply static obstacle conversion plugin
        setStaticCostmap(static_costmap);
        convertStaticObstacles();

        // Store converted static obstacles for publishing
        auto static_polygons = getStaticPolygons();
        for (auto it = static_polygons->begin(); it != static_polygons->end(); ++it)
        {
          obstacles->obstacles.emplace_back();
          obstacles->obstacles.back().polygon = *it;
          obstacles->obstacles.back().velocities.twist.linear.x = 0;
          obstacles->obstacles.back().velocities.twist.linear.y = 0;
          obstacles->obstacles.back().id = -1;
        }
      }
      // Otherwise leave static obstacles point-shaped
      else
      {
        for (int i = 0; i < height; i++)
        {
          for (int j = 0; j < width; j++)
          {
            uchar value = img_data[i * stride + j];
            if (value > 0)
            {
              // obstacle found
              obstacles->obstacles.emplace_back();
              geometry_msgs::msg::Point32 pt;
              pt.x = (double)j * costmap_->getResolution() + costmap_->getOriginX();
              pt.y = (double)i * costmap_->getResolution() + costmap_->getOriginY();
              obstacles->obstacles.back().polygon.points.push_back(pt);
              obstacles->obstacles.back().velocities.twist.linear.x = 0;
              obstacles->obstacles.back().velocities.twist.linear.y = 0;
              obstacles->obstacles.back().id = -1;
            }
          }
        }
      }
    }

    updateObstacleContainer(obstacles);
  }

  void CostmapToDynamicObstacles::setCostmap2D(nav2_costmap_2d::Costmap2D *costmap)
  {
    if (!costmap)
      return;

    costmap_ = costmap;

    updateCostmap2D();
  }

  void CostmapToDynamicObstacles::updateCostmap2D()
  {
    if (!costmap_->getMutex())
    {
      RCLCPP_ERROR(getLogger(), "Cannot update costmap since the mutex pointer is null");
      return;
    }
    std::unique_lock<nav2_costmap_2d::Costmap2D::mutex_t> lock(*costmap_->getMutex());

    // Initialize costmapMat_ directly with the unsigned char array of costmap_
    // costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
    //                      costmap_->getCharMap()).clone(); // Deep copy: TODO
    costmap_mat_ = cv::Mat(costmap_->getSizeInCellsX(), costmap_->getSizeInCellsY(), CV_8UC1,
                           costmap_->getCharMap());
  }

  ObstacleArrayConstPtr CostmapToDynamicObstacles::getObstacles()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    return obstacles_;
  }

  void CostmapToDynamicObstacles::updateObstacleContainer(ObstacleArrayPtr obstacles)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    obstacles_ = obstacles;
  }

  Point_t CostmapToDynamicObstacles::getEstimatedVelocityOfObject(unsigned int idx)
  {
    // vel [px/s] * costmapResolution [m/px] = vel [m/s]
    Point_t vel = tracker_->tracks.at(idx)->getEstimatedVelocity() * costmap_->getResolution() + ego_vel_;

    // RCLCPP_WARN(getLogger(), "Estimated Tracked Object vel x: %f, vel y: %f, vel z: %f", vel.x, vel.y, vel.z);

    // velocity in /map frame
    return vel;
  }

  void CostmapToDynamicObstacles::odomCallback(const nav_msgs::msg::Odometry::ConstSharedPtr msg)
  {
    RCLCPP_INFO_ONCE(getLogger(), "CostmapToDynamicObstacles: odom received.");

    tf2::Quaternion pose;
    tf2::fromMsg(msg->pose.pose.orientation, pose);

    tf2::Vector3 twistLinear;
    // tf2::fromMsg(msg->twist.twist.linear, twistLinear); // not available in tf2
    twistLinear.setX(msg->twist.twist.linear.x);
    twistLinear.setY(msg->twist.twist.linear.y);
    twistLinear.setZ(msg->twist.twist.linear.z);

    // velocity of the robot in x, y and z coordinates
    tf2::Vector3 vel = tf2::quatRotate(pose, twistLinear);
    ego_vel_.x = vel.x();
    ego_vel_.y = vel.y();
    ego_vel_.z = vel.z();
  }

  void CostmapToDynamicObstacles::getContour(unsigned int idx, std::vector<Point_t> &contour)
  {
    assert(!tracker_->tracks.empty() && idx < tracker_->tracks.size());

    contour.clear();

    // contour [px] * costmapResolution [m/px] = contour [m]
    std::vector<cv::Point> contour2i = tracker_->tracks.at(idx)->getLastContour();

    contour.reserve(contour2i.size());

    Point_t costmap_origin(costmap_->getOriginX(), costmap_->getOriginY(), 0);

    for (std::size_t i = 0; i < contour2i.size(); ++i)
    {
      contour.push_back((Point_t(contour2i.at(i).x, contour2i.at(i).y, 0.0) * costmap_->getResolution()) + costmap_origin); // Shift to /map
    }
  }

  void CostmapToDynamicObstacles::visualize(const std::string &name, const cv::Mat &image)
  {
    if (!image.empty())
    {
      cv::Mat im = image.clone();
      cv::flip(im, im, 0);
      cv::imshow(name, im);
      cv::waitKey(1);
    }
  }
}
