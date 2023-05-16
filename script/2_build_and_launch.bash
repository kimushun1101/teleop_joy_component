#!/bin/bash
cd `dirname $0`/../../.. # ros2 workspace directory
colcon build
. install/local_setup.bash
ros2 launch teleop_joy teleop_joy_launch.py