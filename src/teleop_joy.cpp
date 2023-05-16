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

#include "teleop_joy/teleop_joy.hpp"

namespace teleop_joy
{
TeleopJoy::TeleopJoy(const rclcpp::NodeOptions & options)
: Node("teleop_joy", options)
{
  this->declare_parameter<std::string>("assignment_file", "assignment.yaml");
  this->declare_parameter<std::string>("cmd_vel_topic_name", "cmd_vel");
  this->declare_parameter<double>("max.v", 1.0);
  this->declare_parameter<double>("max.w", 1.0);

  auto path = ament_index_cpp::get_package_share_directory("teleop_joy") + "/config/" +
    this->get_parameter("assignment_file").as_string();
  try {
    RCLCPP_INFO_STREAM(this->get_logger(), "Success to open " << path);
    input_ = YAML::LoadFile(path);
  } catch (...) {
    RCLCPP_ERROR_STREAM(this->get_logger(), "Failed to open " << path);
    RCLCPP_ERROR_STREAM(
      this->get_logger(), "Edit 'assignment_file' in 'config/param.yaml' then colcon build");
  }

  auto cmd_vel_topic_name = this->get_parameter("cmd_vel_topic_name").as_string();
  sub_ =
    create_subscription<sensor_msgs::msg::Joy>(
    "joy", 1,
    std::bind(&TeleopJoy::joy_callback, this, std::placeholders::_1));
  pub_ = create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic_name, 1);
}

void TeleopJoy::joy_callback(sensor_msgs::msg::Joy::ConstSharedPtr msg)
{
  // Publish cmd_vel
  geometry_msgs::msg::Twist cmd_vel;
  if (msg->buttons[input_["L_trigger_1"].as<uint8_t>()]) {
    auto v_max = this->get_parameter("max.v").as_double();
    auto w_max = this->get_parameter("max.w").as_double();
    cmd_vel.linear.x = v_max * msg->axes[input_["L_stick_ver"].as<uint8_t>()];
    cmd_vel.linear.y = v_max * msg->axes[input_["L_stick_hoz"].as<uint8_t>()];
    cmd_vel.angular.z = w_max * msg->axes[input_["R_stick_hoz"].as<uint8_t>()];
    RCLCPP_DEBUG_STREAM(
      this->get_logger(), "[linear.x, linear.y, angular.z] = ["
        << cmd_vel.linear.x << ", " << cmd_vel.linear.y << ", " << cmd_vel.angular.z << "]");
  }
  this->pub_->publish(cmd_vel);

  // Template for detecting a button
  if (msg->buttons[input_["L_btn_right"].as<uint8_t>()]) {
    RCLCPP_DEBUG_STREAM(
      this->get_logger(),
      "Pressed " << "L_btn_right" << " : button " << input_["L_btn_right"]);
  }

  // Debug
  std::vector<std::string> axis_names_ = {
    "L_stick_hoz",
    "L_stick_ver",
    "R_stick_hoz",
    "R_stick_ver",
    "L_cross_hoz",
    "L_cross_ver"
  };
  for (const auto & n : axis_names_) {
    if (msg->axes[input_[n].as<uint8_t>()]) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Pressed " << n << " : axis " << input_[n]);
    }
  }
  std::vector<std::string> buttons_names_ = {
    "L_btn_left",
    "L_btn_right",
    "L_btn_up",
    "L_btn_down",
    "L_trigger_1",
    "L_trigger_2",
    "R_trigger_1",
    "R_trigger_2",
    "L_stick_push",
    "R_stick_push"
  };
  for (const auto & n : buttons_names_) {
    if (msg->buttons[input_[n].as<uint8_t>()]) {
      RCLCPP_DEBUG_STREAM(this->get_logger(), "Pressed " << n << " : button " << input_[n]);
    }
  }
}

}  // namespace teleop_joy

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(teleop_joy::TeleopJoy)
