#pragma once

#include <string>
#include <vector>
#include <cstdint>
#include <algorithm>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp/node.hpp"
#include "rclcpp/publisher.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/string.hpp"

#include <phidget22.h>

namespace phidgets_hardware
{

class PhidgetsBldcHardware : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(PhidgetsBldcHardware)

  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;
  hardware_interface::CallbackReturn on_activate(const rclcpp_lifecycle::State &) override;
  hardware_interface::CallbackReturn on_deactivate(const rclcpp_lifecycle::State &) override;
  hardware_interface::return_type read(const rclcpp::Time &, const rclcpp::Duration &) override;
  hardware_interface::return_type write(const rclcpp::Time &, const rclcpp::Duration &) override;

private:
  // Dynamic configuration (auto-detected from URDF)
  std::vector<int32_t> device_serial_;
  std::vector<int> hub_port_;
  std::vector<int> channel_;
  std::vector<double> direction_sign_;
  std::vector<std::string> joint_names_;
  std::vector<bool> is_left_wheel_;

  // Motor state tracking
  std::vector<bool> motor_enabled_;
  std::vector<bool> connection_attempted_;
  std::vector<bool> attached_;
  std::vector<bool> temperature_attached_;
  
  // Phidget handles
  std::vector<PhidgetBLDCMotorHandle> motors_;
  std::vector<PhidgetTemperatureSensorHandle> temperature_sensors_;
  
  // Data storage
  std::vector<double> pos_rad_;
  std::vector<double> vel_state_;
  std::vector<double> temperature_c_;
  std::vector<double> cmd_;
  
  // Configuration parameters
  double command_limit_{1.0};
  double acceleration_{5.0};
  double stall_velocity_{0.0};
  double gear_ratio_{106.0};
  int commutations_per_motor_rev_{24};
  double rescale_factor_rot_{1.0};

  // ROS Communication
  rclcpp::Node::SharedPtr telemetry_node_;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr motor_telemetry_pub_;
  rclcpp::Time last_telemetry_publish_time_{0, 0, RCL_ROS_TIME};
  double telemetry_publish_period_sec_{0.2};

  static double clamp(double x, double lo, double hi) {
    return (x < lo) ? lo : ((x > hi) ? hi : x);
  }

  void setup_ros_communication();
  void publish_motor_telemetry();
  void connect_all_motors();
  void connect_motor(int i);
  void disable_motor(int i, const std::string& reason);
  void close_phidget(int i);
  void close_temperature_sensor(int i);
  void try_attach_motor(int i);
  void try_attach_temperature_sensor(int i);
  bool is_left_wheel_joint(const std::string& joint_name);
};

}  // namespace phidgets_hardware