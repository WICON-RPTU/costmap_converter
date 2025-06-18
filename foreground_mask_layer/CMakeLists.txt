cmake_minimum_required(VERSION 3.5)
project(foreground_mask_layer)

set(lib_name ${PROJECT_NAME})

# === Environment ===

# Default to C99
if(NOT CMAKE_C_STANDARD)
  set(CMAKE_C_STANDARD 99)
endif()

# Default to C++14
if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# === Dependencies ===

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav2_costmap_2d REQUIRED)
find_package(pluginlib REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

set(dep_pkgs
    rclcpp
    nav2_costmap_2d
    tf2
    tf2_ros
    tf2_geometry_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    pluginlib)

# === Build ===

add_library(foreground_mask_layer SHARED
  src/foreground_mask_layer.cpp
)
include_directories(include)

add_executable(dynamic_obstacle_node src/odpp_node.cpp)
ament_target_dependencies(dynamic_obstacle_node ${dep_pkgs})

# === Installation ===

install(TARGETS ${lib_name}
        DESTINATION lib)

install(TARGETS dynamic_obstacle_node
  DESTINATION lib/${PROJECT_NAME}
)

# Install required directories (e.g. launch, config)
install(DIRECTORY 
  launch
  DESTINATION share/${PROJECT_NAME}
)

# === Ament work ===

# pluginlib_export_plugin_description_file() installs gradient_layer.xml
# file into "share" directory and sets ament indexes for it.
# This allows the plugin to be discovered as a plugin of required type.
pluginlib_export_plugin_description_file(nav2_costmap_2d
  foreground_mask.xml)
ament_target_dependencies(foreground_mask_layer ${dep_pkgs})
ament_package()