from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import TextSubstitution
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    file_name_arg = DeclareLaunchArgument(
        "file_name", default_value=TextSubstitution(text="assignment.yaml")
    )
    joy_node = Node(
        package='joy',
        executable='joy_node',
    )
    config_joy_node = Node(
        package='teleop_joy',
        executable='config_joy',
        parameters=[{"file_name": LaunchConfiguration('file_name')}]
    )
    return LaunchDescription([
        file_name_arg,
        joy_node,
        config_joy_node
    ])