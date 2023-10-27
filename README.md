# teleop_joy_component package
This package translates `/joy` topic into `/cmd_vel` topic on **ROS 2 Composition**.  
The following 2 compositions are launched.
- joy joy_node
- teleop_joy_component joy_to_cmd_vel_composition

## How to use it
1. (Optional) `Use this template` and `Create a new repository`.
2. `git clone` into `${ROS_WORKSPACE}/src`.
    Replace `ROS_WORKSPACE` and `GitHub URL`, then command the following.
    ```
    export ROS_WORKSPACE=$HOME/ros2_ws
    mkdir -p $ROS_WORKSPACE/src && cd $ROS_WORKSPACE/src
    git clone git@github.com:kimushun1101/teleop_joy_component_template.git
    rosdep install -riy --from-paths $ROS_WORKSPACE/src/teleop_joy_component_template
    ```
3. `cd ${THIS_PKG_DIR}`, then execute scripts in `script` directory.
    ```
    ./script/1_rename_pkg_name_into_the_same_as_directory_name
    ./script/2_make_new_assignment.bash
    ./script/3_rebuild_and_launch.bash
    ```

## Reference
- [teleop_twist_joy](https://github.com/ros2/teleop_twist_joy)
  : Similar package
- [About Composition](https://docs.ros.org/en/humble/Concepts/About-Composition.html)
  : Concept of ROS 2 Composition
- [Source Code Sample](https://github.com/ros2/demos/tree/humble/composition)
  : Basis of this package
- [ROS 2 Tutorial](https://docs.ros.org/en/humble/Tutorials/Intermediate/Composition.html)
  : Command line interfaces for ROS 2 Components
- [ROS Handson (in Japanese)](https://ouxt-polaris.github.io/ros_handson/rclcpp)
  : Japanese description about ROS 2 Composition

## License
Apache-2.0