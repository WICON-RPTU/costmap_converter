import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the package directories
    foreground_mask_layer_share = get_package_share_directory('foreground_mask_layer')

    # Declare Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='/fleet/skid_steered_two_lidars_0/',
        description='Top-level namespace')
    
    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(foreground_mask_layer_share, 'config', 'odpp.yaml'),
        description='Full path to the ROS2 parameters file')
    
    # Setting Launch Configuration
    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')


    # Description of Nodes
    odpp_node = Node(
            package='foreground_mask_layer',
            executable='dynamic_obstacle_node',
            name='dynamic_obstacle_node',
            output='screen',
            namespace = namespace,
            parameters = [params_file],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')],
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(odpp_node)
    return ld

