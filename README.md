# `costmap_converter` ROS Package

The `costmap_converter` package provides plugins and nodes for converting occupied `costmap_2d` cells into primitive geometric shapes or dynamic obstacles. It is designed to enhance navigation frameworks by offering flexible costmap data processing.


## Table of Contents

- [Build Status](#build-status)
- [Features](#features)
- [Major Updates in This Fork](#major-updates-in-this-fork)
- [Installation](#installation)
  - [Prerequisites](#prerequisites)
  - [Build Instructions](#build-instructions)
- [Configuration](#configuration)
  - [Example Configuration File](#example-configuration-file)
  - [Updates to the Codebase](#updates-to-the-codebase)
- [Usage](#usage)
  - [Launching the Nodes](#launching-the-nodes)
  - [Parameter Customization](#parameter-customization)
- [Contributors](#contributors)
- [License](#license)
  - [Third-Party Dependencies](#third-party-dependencies)
- [Acknowledgments](#acknowledgments)
- [Contact](#contact)

---


## Build Status

Build status for the **humble-dev** branch:

- *Unavailable*
---

## Features

1. **Static Costmap Conversion**: Converts occupied cells to simple geometric shapes such as polygons and circles.
2. **Dynamic Obstacle Detection**:
   - Includes a `CostmapToDynamicObstacles` plugin for detecting and tracking dynamic obstacles.
   - Implements advanced techniques such as background subtraction, blob detection, and multi-target tracking.
3. **ROS2 Compatibility**: Updated to support dynamic parameter reconfiguration in ROS2.
4. **Customizable Parameters**: Fully configurable plugins and nodes for user-specific requirements.

---

## Major Updates in This Fork

- **Improved Plugin Flexibility**: Refactored code to enhance modularity and extend support for advanced costmap conversion tasks.
- **ROS2 Support**: Fully transitioned to ROS2 while maintaining compatibility with legacy configurations.
- **Dynamic Parameterization**: 
  - Enables real-time parameter adjustments for background subtraction, blob detection, and multi-target tracking; for Dynamic Obstacle Plugin

---

## Installation

### Prerequisites

Ensure that you have the following installed:
- ROS2 Humble (or your target ROS2 distribution)
- OpenCV (version >= 3.2.0)
- Other dependencies listed in `package.xml`

### Build Instructions

1. Clone the repository in your **/src** folder:

   - Using **HTTPS**:
     ```bash
     git clone -b humble_dev https://github.com/WICON-RPTU/costmap_converter.git
     ```

   - Using **SSH**:
     ```bash
     git clone -b humble_dev git@github.com:WICON-RPTU/costmap_converter.git
     ```

2. Build the workspace:
   ```bash
   # from the root of your workspace
   colcon build
   ```
3. Source the setup file:
   ```bash
   source install/setup.bash
   ```
---


## Configuration

The `costmap_converter` package supports a wide range of parameters to fine-tune the behavior of dynamic obstacle detection, extraction, and tracking. Below is an example configuration file showcasing these parameters:

```yaml
/**/intra_node:
  ros__parameters:
    # Static Layer: PolygonDBSMCCH #################
    PolygonDBSMCCH:
      cluster_max_distance: 0.2                   # Maximum distance to neighbors [m]
      cluster_max_pts: 50                         # Maximum number of points that define a cluster
      cluster_min_pts: 5                          # Minimum number of points that define a cluster
      convex_hull_min_pt_separation: 0.0          # Clear keypoints of the convex polygon that are close to each other [distance in meters]

    # Costmap Converter Dynamic Obstacle Plugin
    dynamic_obstacle_plugin:
      publish_static_obstacles: true
      static_converter_plugin: costmap_converter::CostmapToPolygonsDBSMCCH

    # Foreground Detection
    bg_sub_params:
      alpha_slow: 0.9                             # Learning rate of the slow filter
      alpha_fast: 1.0                             # Learning rate of the fast filter
      beta: 0.9                                   # Weighting coefficient for pixel's value
      min_sep_between_fast_and_slow_filter: 0.1   # Minimal difference to recognize dynamic obstacles
      min_occupancy_probability: 220.0            # Minimum occupancy probability to classify dynamic obstacles
      max_occupancy_neighbors: 40.0               # Maximum neighbor occupancy value for dynamic classification
      morph_size: 6                               # Structuring element size for closing operation

    # Blob Detection
    blob_det_params:
      filter_by_color: true                       # Always true to filter blobs by intensity
      blob_color: 255                             # Extract light blobs
      threshold_step: 256.0                       # Distance between thresholds
      min_threshold: 127.0                        # Minimum threshold for binary image conversion
      max_threshold: 255.0                        # Maximum threshold for binary image conversion
      min_repeatability: 1                        # Minimum blob detections for validity
      min_distance_between_blobs: 200.0           # Minimum distance between blob centers
      filter_by_area: true                        # Filter blobs by pixel count
      min_area: 10.0                              # Minimum blob size in pixels
      max_area: 200.0                             # Maximum blob size in pixels
      filter_by_circularity: true                 # Filter blobs by circularity
      min_circularity: 0.2                        # Minimum circularity value
      max_circularity: 1.0                        # Maximum circularity value
      filter_by_inertia: true                     # Filter blobs by inertia ratio
      min_inertia_ratio: 0.2                      # Minimum inertia ratio
      max_inertia_ratio: 0.8                      # Maximum inertia ratio
      filter_by_convexity: false                  # Filter blobs by convexity
      min_convexity: 0.0                          # Minimum convexity ratio
      max_convexity: 1.0                          # Maximum convexity ratio

    # Tracking
    tracker_params:
      dt: 0.1                                     # Time for one timestep of the Kalman filter
      dist_thresh: 150.0                          # Maximum assignment distance threshold
      max_allowed_skipped_frames: 3               # Maximum frames to track without detection
      max_trace_length: 100                       # Maximum trace length

    use_sim_time: true
```

### Updates to the Codebase

The dynamic parameterization feature has been integrated to allow runtime updates only for key *Dynamic Obstacle* components:
1. **Background Subtractor Parameters**:
   - Fully dynamic updates for learning rates (`alpha_slow`, `alpha_fast`), thresholds, and morphological settings.
2. **Blob Detector Parameters**:
   - Real-time control over blob detection properties such as `threshold_step`, `min_area`, and `max_area`.
3. **Tracker Parameters**:
   - Adjustable tracking parameters like `dt`, `dist_thresh`, and `max_trace_length`.

These updates enhance the flexibility and adaptability of the `costmap_converter` package, making it highly configurable for various robotic applications.

---


## Usage

The `costmap_converter` package provides plugins and standalone nodes to convert costmap data into static & dynamic obstacles. Below are instructions for launching and integrating the package:

### Launching the Nodes

1. **Standalone Demonstration with Navigation2 and TurtleBot3 Simulation**  
   This launch file demonstrates the `costmap_converter` plugin in a TurtleBot3 simulation environment using the Navigation2 stack:
   ```bash
   ros2 launch costmap_converter example_standalone_converter.launch.py
   ```
   - Launches a full TurtleBot3 simulation.
   - Integrates the `costmap_converter` plugin to showcase obstacle detection and conversion.
   - Preconfigured with parameters in the `config/standalone_converter.yaml` file.
   - In the RViz2 window, after setting up NAV2, add a display for `costmap_polygon_markers/Marker` under the `By topic` tab to vizaulize the polygons. 

2. **Integration into Existing ROS2 Launch Sequences**  
   Use this launch file to integrate the `costmap_converter` plugin into your custom ROS2 launch setup:
   ```bash
   ros2 launch costmap_converter standalone_converter.launch.py
   ```
   - Flexible integration options:
     - **Namespace**: Organize nodes into specific namespaces.
     - **Parameter File**: Specify custom configurations.
   - Example with namespace and a custom parameter file:
     ```bash
     ros2 launch costmap_converter standalone_converter.launch.py namespace:=robot1 params_file:=/path/to/custom_params.yaml
     ```

### Parameter Customization

Parameters can be updated dynamically during runtime using ROS2’s parameter server:
```bash
ros2 param set <node_name> <parameter_name> <value>
```
Example:
```bash
ros2 param set /standalone_converter_node bg_sub_params.alpha_slow 0.5
```
---

## Contributors

- **[Christoph Rösmann](https://github.com/croesmann)**  
- **[Franz Albers](https://github.com/FranzAlbers)** (*CostmapToDynamicObstacles* plugin)  
- **Otniel Rinaldo**  
- **[Michael Schröder](https://github.com/M-Schroeder)** (*Proposed and co-implemented example launch sequence*)
- **[Riyan Cyriac Jose](https://github.com/joseriyancyriac)** (*ROS2 adaptation, dynamic parameterization, and co-implemented example launch sequence*)  

---

## License

The *costmap_converter* package is licensed under the BSD license. It depends on other ROS packages, which are also BSD licensed, as specified in the `package.xml`.

### Third-Party Dependencies

- **MultitargetTracker**: Licensed under GNU GPLv3 (https://github.com/Smorodov/Multitarget-tracker). Used partially in the `CostmapToDynamicObstacles` plugin.

This package and its components are distributed with the hope that they will be useful but **WITHOUT ANY WARRANTY**. See the licenses for more details.

---

## Acknowledgments

This work is supported by:
- **Department of Wireless Communication and Navigation (WICON), RPTU Kaiserslautern**  
- **Department of Electromobility (LEM), RPTU Kaiserslautern**

The repository is actively maintained to support academic research and development.

---

## Contact

For bug reports, feature requests, or contributions, open an issue on [GitHub](https://github.com/<your-github-username>/costmap_converter).

---
