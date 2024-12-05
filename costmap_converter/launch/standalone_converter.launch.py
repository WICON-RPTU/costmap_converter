import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():

    # Get the package directories
    fleet_share = get_package_share_directory('costmap_converter')

    # Load all parameters
    standalone_converter_config = os.path.join(fleet_share, 'config', 'standalone_converter.yaml')

    # Declare Launch Arguments
    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='',
        description='Top-level namespace')
    
    # Setting Launch Configuration
    namespace = LaunchConfiguration('namespace')


    # Description of Nodes
    costmap_converter_node = Node(
            package ='costmap_converter',
            executable ='standalone_converter',
            # namespace = namespace,
            output = 'screen',
            parameters = [standalone_converter_config],
            remappings=[
                ('/tf', 'tf'),
                ('/tf_static', 'tf_static')],
    )

    # Launch Description
    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(costmap_converter_node)
    return ld

