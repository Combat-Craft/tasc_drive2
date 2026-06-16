#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/string.hpp"

#include <cmath>
#include <string>

class Ps4Teleop : public rclcpp::Node
{
public:
  Ps4Teleop() : Node("ps4_teleop")
  {
    joy_sub_ = this->create_subscription<sensor_msgs::msg::Joy>(
        "/joy",
        10,
        std::bind(&Ps4Teleop::joy_callback, this, std::placeholders::_1));

    cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>(
        "/diff_drive_controller/cmd_vel_unstamped",
        10);

    relay_state_sub_ = this->create_subscription<std_msgs::msg::String>(
        "/rover/relay_board/state",
        10,
        std::bind(&Ps4Teleop::relay_state_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "PS4 teleop node started");
  }

private:
  double apply_deadzone(double value, double deadzone, double expo = 1.8) const
  {
    const double magnitude = std::fabs(value);
    if (magnitude <= deadzone) {
      return 0.0;
    }

    const double normalized = (magnitude - deadzone) / (1.0 - deadzone);
    const double shaped = std::pow(normalized, expo);
    return std::copysign(shaped, value);
  }

  void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg)
  {
    geometry_msgs::msg::Twist cmd;

    const double forward_input = apply_deadzone(msg->axes[1], 0.18); // left stick vertical
    const double turn_input = apply_deadzone(msg->axes[3], 0.18);    // right stick horizontal

    const bool boost_enabled = msg->buttons.size() > 5 && msg->buttons[5] == 1;
    const double speed_scale = boost_enabled ? 1.0 : 0.5;
    const double turn_scale = boost_enabled ? 1.0 : 0.5;

    // Match diff_drive outputs to the normalized [-1, 1] motor command budget.
    const double max_linear = 0.0925 * speed_scale;
    const double max_angular = 0.474 * turn_scale;

    if (all_relays_enabled_) {
      cmd.linear.x = forward_input * max_linear;
      cmd.angular.z = turn_input * max_angular;
    } else {
      cmd.linear.x = 0.0;
      cmd.angular.z = 0.0;
    }

    cmd_pub_->publish(cmd);
  }

  bool relay_enabled(const std::string & payload, const std::string & joint_name) const
  {
    const std::string needle = "\"name\":\"" + joint_name + "\"";
    const auto name_pos = payload.find(needle);
    if (name_pos == std::string::npos) {
      return false;
    }

    const auto object_end = payload.find('}', name_pos);
    if (object_end == std::string::npos) {
      return false;
    }

    const auto enabled_pos = payload.find("\"enabled\":true", name_pos);
    return enabled_pos != std::string::npos && enabled_pos < object_end;
  }

  void relay_state_callback(const std_msgs::msg::String::SharedPtr msg)
  {
    const std::string payload = msg->data;
    all_relays_enabled_ =
      relay_enabled(payload, "front_left_wheel_joint") &&
      relay_enabled(payload, "middle_left_wheel_joint") &&
      relay_enabled(payload, "rear_left_wheel_joint") &&
      relay_enabled(payload, "middle_right_wheel_joint") &&
      relay_enabled(payload, "front_right_wheel_joint") &&
      relay_enabled(payload, "rear_right_wheel_joint");
  }

  rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr relay_state_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
  bool all_relays_enabled_{false};
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<Ps4Teleop>());
  rclcpp::shutdown();
  return 0;
}
