import os
from ament_index_python import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

param_file = os.path.join(get_package_share_directory('teleop_joy'), 'config', 'param.yaml')


def generate_launch_description():
    container = ComposableNodeContainer(
        name='joy_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package='joy',
                plugin='joy::Joy',
                name='joy'),
            ComposableNode(
                package='teleop_joy',
                plugin='teleop_joy::TeleopJoy',
                name='teleop_joy',
                parameters=[param_file])
        ],
        output='screen',
    )

    return LaunchDescription([
        container,
    ])
