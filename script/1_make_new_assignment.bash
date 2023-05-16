#!/bin/bash
if [ $# -eq 1 ]; then
  cd `dirname $0`
  script_dir=$(pwd)

  cd $script_dir/../../.. # ros2 workspace directory
  colcon build
  . install/local_setup.bash
  cd $script_dir/../config # config directory
  ros2 launch teleop_joy config_joy_launch.py file_name:=$1
else
  echo "Set the file name in the argument."
  echo "ex.) ./1_make_new_assignment.bash assinment.yaml"
fi
