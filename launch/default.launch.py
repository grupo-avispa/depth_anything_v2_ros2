#!/usr/bin/env python3

'''
    Launches a depth_anything_v2_ros2 node with default parameters.
'''
import os

from ament_index_python import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from nav2_common.launch import RewrittenYaml

def generate_launch_description():
    # Getting directories and launch-files
    depth_anything_dir = get_package_share_directory('depth_anything_v2_ros2')
    default_params_file = os.path.join(depth_anything_dir, 'config', 'params.yaml')
    default_model_file = os.path.join(depth_anything_dir, 'models', 'depth_anything_v2_vits.pth')  

    # Input parameters declaration
    params_file = LaunchConfiguration('params_file')
    model_file = LaunchConfiguration('model_file')
    log_level = LaunchConfiguration('log-level')

    declare_params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value = default_params_file,
        description = 'Full path to the ROS2 parameters file with detection configuration'
    )

    declare_model_file_arg = DeclareLaunchArgument(
        'model_file',
        default_value = default_model_file,
        description = 'Full path to the model file'
    )

    declare_log_level_arg = DeclareLaunchArgument(
        name = 'log_level',
        default_value = 'info',
        description = 'Logging level (info, debug, ...)'
    )

    # Create our own temporary YAML files that include substitutions
    param_substitutions = {
        'model_file': model_file,
    }

    configured_params = RewrittenYaml(
        source_file = params_file,
        root_key = '',
        param_rewrites = param_substitutions,
        convert_types = True
    )

    # Prepare the detection node.
    depth_anything_node = Node(
        package = 'depth_anything_v2_ros2',
        namespace = '',
        executable = 'depth_anything_v2_ros2',
        name = 'depth_anything',
        parameters = [configured_params],
        emulate_tty = True,
        output = 'screen', 
        arguments = [
            '--ros-args', 
            '--log-level', ['depth_anything:=', LaunchConfiguration('log_level')]]
    )

    return LaunchDescription([
        declare_params_file_arg,
        declare_model_file_arg,
        declare_log_level_arg,
        depth_anything_node
    ])