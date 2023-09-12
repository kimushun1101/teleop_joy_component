# Copyright 2023 Shunsuke Kimura
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


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
