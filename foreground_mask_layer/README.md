# Costmap2D Plugin:Foreground Mask Layer (ROS 2)

A ROS 2 costmap plugin layer for detecting dynamic obstacles by comparing live sensor data (fast filter) against an inflated static map (slow filter). Designed to work with rolling costmaps and compensate for odometry drift using TF2 transformations.

---

## Package Overview

This package implements a **Foreground Mask Layer** plugin for ROS 2 navigation costmaps (`nav2_costmap_2d::Layer`). It provides:

- Inflation of the static map on reception to handle localization drift and uncertainty.
- Real-time comparison of sensor-influenced costmaps with the inflated static map.
- Foreground mask publication or direct costmap modification with dynamic obstacle information.

---

## Repository Structure

```bash
foreground_mask_layer/
├── include/
│   └── foreground_mask_layer.hpp
├── src/
│   └── foreground_mask_layer.cpp
├── plugin.xml
├── CMakeLists.txt
├── package.xml
└── README.md

````

---

## Getting Started

### Prerequisites

- ROS 2 Humble (or later)
- A workspace set up for ROS 2 (`colcon` build system)

### Installation

Clone the repository into your ROS 2 workspace `src` folder:

```bash
cd ~/your_ros2_ws/src
git clone <this-repo-url>
````

### Build the Package

```bash
cd ~/your_ros2_ws
colcon build --packages-select foreground_mask_layer
source install/setup.bash
```

--

## Configuration

>[!IMPORTANT]
>Add the plugin to your `local_costmap` (or any costmap) YAML config:
>
>```yaml
>local_costmap:
>  plugins: ["obstacle_layer",  "denoise_layer", "foreground_mask_layer"]
>  foreground_mask_layer:
>    plugin: "foreground_mask::ForegroundMaskLayer"
>    publish_mask_only: true         # also publishes /foreground_mask
>    overwrite_costmap: true         # keep original costs
>    mask_cost_value: 255            # Set the costmap value for occupied cells (LETHAL_OBSTACLE = 255)
>    map_topic: "/fleet/skid_steered_two_lidars_0/map"
>    inflation_radius: 10            #Inflate the static map region by this radius
>```

---

## How It Works

### Filter Analogy

| Filter      | Source              | Purpose                          |
| ----------- | ------------------- | -------------------------------- |
| Slow Filter | Inflated static map | Baseline obstacle memory         |
| Fast Filter | Obstacle layer      | Real-time dynamic sensor updates |

### Processing Flow

1. **Static Map Inflation**
   Upon receiving the static map, the plugin inflates it using a configurable radius.

2. **Real-Time Comparison**
   For each cell update:

   * Transform the cell from the rolling `odom` frame to the global `map` frame via TF.
   * Compare costmap value against the inflated static map.
   * If a mismatch is found → mark as foreground.

3. **Output Modes**

   * Publishes a `/foreground_mask` topic (`nav_msgs/msg/OccupancyGrid`)
   * Optionally writes directly into the costmap

---

## Parameters

| Parameter           | Type   | Default | Description                                   |
| ------------------- | ------ | ------- | --------------------------------------------- |
| `publish_mask_only` | bool   | true    | Whether to publish foreground mask separately |
| `overwrite_costmap` | bool   | false   | Whether to modify costmap with mask           |
| `mask_cost_value`   | int    | 255     | Cost value to write into costmap              |
| `map_topic`         | string | `/map`  | Static map topic name                         |
| `inflation_radius`  | float  | 5       | Radius used to inflate the static map         |

---

## Output

* **Topic**: `/foreground_mask`
  Type: `nav_msgs/msg/OccupancyGrid`
  Description: Binary map of detected dynamic obstacles

---

## Example Launch Snippet

Include the plugin in your launch file stack and ensure TFs (`map` → `odom`) are being broadcast properly (e.g., via AMCL).

>[!IMPORANT]
>Also, valid topic names must be updated in the config file such as odometry and scan messages (alongwith namespace consideration)

---

## Contact

If you encounter issues or have feature requests, feel free to open an issue or discussion.

---

## License

BSD 3-Clause License (inherited from `nav2_costmap_2d`)

---
## Author

- Riyan Cyriac Jose: [lul42xex@rptu.de](mailto:lul42xex@rptu.de)