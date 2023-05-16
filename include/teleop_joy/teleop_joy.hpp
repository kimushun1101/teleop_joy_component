// Copyright 2023 Shunsuke Kimura
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#ifndef JOY_TO_CMD_VEL_COMPONENT_HPP_
#define JOY_TO_CMD_VEL_COMPONENT_HPP_

#include "teleop_joy/visibility_control.h"
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
#include <yaml-cpp/yaml.h>

namespace teleop_joy
{

class TeleopJoy : public rclcpp::Node
{
public:
  COMPOSITION_PUBLIC
  explicit TeleopJoy(const rclcpp::NodeOptions & options);

private:
  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
  void joy_callback(sensor_msgs::msg::Joy::ConstSharedPtr msg);

  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  YAML::Node input_;
};

}  // namespace teleop_joy

#endif  // JOY_TO_CMD_VEL_COMPONENT_HPP_
