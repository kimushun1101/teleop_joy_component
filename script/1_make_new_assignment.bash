#!/bin/bash
if [ $# -eq 1 ]; then
  cd `dirname $0`
  script_dir=$(pwd)

  cd $script_dir/../../.. # ros2 workspace directory
  colcon build --symlink-install --packages-select teleop_joy
  . install/local_setup.bash
  cd $script_dir/../config # config directory
  ros2 launch teleop_joy config_joy_launch.py file_name:=$1
  echo -e "\033[32mEdit $(pwd)/param.yaml\033[m"
  echo -e "\033[32mthen Rebuild\033[m"
else
  echo -e "\033[31mSet the file name in the argument\033[m"
  echo -e "\033[31mex.) ./1_make_new_assignment.bash assinment.yaml\033[m"
fi
