# teleop_joy package
This package translates `/joy` topic into `/cmd_vel` topic on **ROS 2 Composition**.  
The following 2 compositions are launched.
- joy joy_node
- teleop_joy joy_to_cmd_vel_composition

## How to use it
1. `git clone` into `your_ros2_workspace/src`.
2. `rosdep install`, `colcon build`, and `source your_ros2_workspace/install/setup.bash`
3. Edit `teleop_joy/config/param.yaml`
4. `ros2 launch teleop_joy teleop_joy.py`

## Configuration `param.yaml`

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