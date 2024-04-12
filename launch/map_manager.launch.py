"""Launch the rover simulation including tf node."""
import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node

from nav2_common.launch import RewrittenYaml

def generate_launch_description():

    # Load Directories
    map_manager_dir = get_package_share_directory('map_manager')

    config_file = LaunchConfiguration('config_file')

    # Create the launch declarations
    declare_config_file_cmd = DeclareLaunchArgument(
        'config_file',
        default_value=os.path.join(map_manager_dir, 'config', 'map_manager_load_config.yaml'),
        description='Params to load and public N static maps')

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
    }

    configured_params = RewrittenYaml(
        source_file=config_file,
        root_key='',
        param_rewrites=param_substitutions,
        convert_types=True)

    # Node to convert odometry message to tf message
    map_manager_cmd = Node(
        package='map_manager',
        executable='map_manager_server',
        name='map_manager_server',
        parameters=[configured_params],
    )
                                 

    return LaunchDescription([
        # Set env var to print messages colored. The ANSI color codes will appear in a log.

        declare_config_file_cmd,
        # Launch Arguments
        # Start Nodes
        map_manager_cmd,
    ])
