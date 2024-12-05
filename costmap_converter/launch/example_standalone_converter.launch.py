import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():

    # Get the package directories
    costmap_converter_share = get_package_share_directory('costmap_converter')
    nav2_share = get_package_share_directory('nav2_bringup')

    # Load all parameters
    standalone_converter_config = os.path.join(costmap_converter_share, 'config', 'standalone_converter.yaml')

    set_env_variable_model_name_cmd = SetEnvironmentVariable(name='TURTLEBOT3_MODEL', value='waffle')
    set_env_variable_path_cmd = SetEnvironmentVariable(name='GAZEBO_MODEL_PATH', value='$GAZEBO_MODEL_PATH:/opt/ros/humble/share/turtlebot3_gazebo/models')
    
    # Description of Launch
    nav2_launch_cmd = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(os.path.join(nav2_share, 'launch', 'tb3_simulation_launch.py')),
        launch_arguments={'headless': 'False'}.items()
    )

    # Description of Nodes
    costmap_converter_node = Node(
            package ='costmap_converter',
            executable ='standalone_converter',
            output = 'screen',
            parameters = [standalone_converter_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')],
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(set_env_variable_model_name_cmd)
    ld.add_action(set_env_variable_path_cmd)
    ld.add_action(nav2_launch_cmd)
    ld.add_action(costmap_converter_node)
    return ld

