#include <memory>
#include <iostream>
#include <fstream>
#include <filesystem>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/joy.hpp>
using std::placeholders::_1;

class ConfigJoy : public rclcpp::Node
{
  public:
    ConfigJoy() : Node("config_joy")
    {
      this->declare_parameter<std::string>("file_name", "assignment.yaml");
      file_name_ = this->get_parameter("file_name").as_string();
      sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "joy", 1, std::bind(&ConfigJoy::joy_callback, this, _1));
      pre_axis_ = -1;
      pre_button_ = -1;
      assign_name_count_ = 0;

      using namespace std::chrono_literals;
      rclcpp::sleep_for(3s);
      RCLCPP_INFO_STREAM(this->get_logger(), "Input '" << assign_name_[assign_name_count_]
        << "' (" << assign_name_count_+1 << "/" << assign_name_.size() << ")");
    }

  private:
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr sub_;
    YAML::Node input_;
    int pre_axis_;
    int pre_button_;
    int assign_name_count_;
    std::string file_name_;
    std::vector<int> assign_resister_;
    std::vector<std::string> assign_name_ = {
      "L_stick_hoz",
      "L_stick_ver",
      "R_stick_hoz",
      "R_stick_ver",
      "L_cross_hoz",
      "L_cross_ver",
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

    void joy_callback(const sensor_msgs::msg::Joy & msg) 
    {
      // search axes
      for(int i = 0; i < static_cast<int>(msg.axes.size()); i++){
        if(std::abs(msg.axes[i]) > 0.9 && i != pre_axis_){
          ConfigJoy::add_assignment(i);
          pre_axis_ = i;
          continue;
        }
      }
      // search buttons
      for(int i = 0; i < static_cast<int>(msg.buttons.size()); i++){
        if(msg.buttons[i] == 1 && i != pre_button_){
          ConfigJoy::add_assignment(i);
          pre_button_ = i;
          continue;
        }
      }
    }

    void add_assignment(const int joy_topic_num)
    {
      if(assign_name_count_ < static_cast<int>(assign_name_.size())){
        // Resister assignments
        RCLCPP_INFO_STREAM(this->get_logger(),
          "Set '" << assign_name_[assign_name_count_] << "' as " << joy_topic_num);
        assign_resister_.push_back(joy_topic_num);
        assign_name_count_++;
      }else{
        return;
      }

      if(assign_name_count_ < static_cast<int>(assign_name_.size())){
        // Prompt the next input
        RCLCPP_INFO_STREAM(this->get_logger(), "Input '" << assign_name_[assign_name_count_]
          << "' (" << assign_name_count_+1 << "/" << assign_name_.size() << ")");
      }
      else if (assign_name_count_ == static_cast<int>(assign_name_.size())){
        YAML::Node assign;
        // Display configuration and save
        RCLCPP_INFO_STREAM(this->get_logger(), "--- Configuration ---");
        for(int i =0; i < assign_name_count_; i++){
          RCLCPP_INFO_STREAM(this->get_logger(), assign_name_[i] << ":" << assign_resister_[i]);
          assign[assign_name_[i]] = assign_resister_[i];
        }
        YAML::Emitter out;
        out << assign;
        auto path = std::filesystem::current_path();
        auto file_path = path.string() + "/" + file_name_;
        std::ofstream file(file_path);
        file << out.c_str();
        file.close();

        RCLCPP_INFO_STREAM(this->get_logger(), "Output file path : " << file_path);
        RCLCPP_INFO_STREAM(this->get_logger(), "Input Ctrl+C, and terminate this program.");
      }
    }
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ConfigJoy>());
  rclcpp::shutdown();
  return 0;
}
