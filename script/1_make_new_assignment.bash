#!/bin/bash

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

if [ $# -eq 1 ]; then
  cd `dirname $0`
  script_dir=$(pwd)

  cd $script_dir/../../.. # ros2 workspace directory
  colcon build --symlink-install --packages-select teleop_joy_component
  . install/local_setup.bash
  cd $script_dir/../config # config directory
  ros2 launch teleop_joy_component config_joy.launch.py file_name:=$1
  echo -e "\033[32mEdit $(pwd)/param.yaml\033[m"
  echo -e "\033[32mthen Rebuild\033[m"
else
  echo -e "\033[31mSet the file name in the argument\033[m"
  echo -e "\033[31mex.) ./1_make_new_assignment.bash assinment.yaml\033[m"
fi
