#!/bin/bash
cd `dirname $0`/../../.. # ros2 workspace directory
colcon build --symlink-install --packages-select teleop_joy
. install/local_setup.bash
ros2 launch teleop_joy teleop_joy_launch.py