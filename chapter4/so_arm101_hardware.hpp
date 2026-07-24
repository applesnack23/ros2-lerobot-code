#ifndef SO_ARM101_DRIVER__SO_ARM101_HARDWARE_HPP_
#define SO_ARM101_DRIVER__SO_ARM101_HARDWARE_HPP_

#include <atomic>
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

#include "hardware_interface/hardware_component_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"

#include "rclcpp/rclcpp.hpp"
#include "rclcpp_lifecycle/state.hpp"
#include "std_msgs/msg/bool.hpp"

namespace so_arm101_driver
{

class SOArm101Hardware : public hardware_interface::SystemInterface
{
public:
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareComponentInterfaceParams & params) override;

  std::vector<hardware_interface::StateInterface>
  export_state_interfaces() override;

  std::vector<hardware_interface::CommandInterface>
  export_command_interfaces() override;

  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  hardware_interface::return_type read(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

  hardware_interface::return_type write(
    const rclcpp::Time & time,
    const rclcpp::Duration & period) override;

private:
  std::string port_ = "/dev/ttyACM0";
  int baudrate_ = 1000000;
  int fd_ = -1;

  std::vector<std::string> joint_names_;
  std::vector<int> motor_ids_;

  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;
  std::vector<double> prev_positions_;

  std::vector<int> last_command_ticks_;

  std::atomic<bool> connected_{false};
  std::atomic<bool> torque_enabled_{false};

  std::mutex bus_mutex_;
  std::mutex state_mutex_;

  rclcpp::Node::SharedPtr torque_node_;
  rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr torque_sub_;

  std::thread torque_spin_thread_;
  std::atomic<bool> torque_spin_running_{false};

  bool open_port();
  void close_port();

  bool send_packet(
    uint8_t id,
    uint8_t instruction,
    const std::vector<uint8_t> & params);

  bool read_status_packet(
    uint8_t expected_id,
    std::vector<uint8_t> & params);

  bool write_byte(
    uint8_t id,
    uint8_t address,
    uint8_t value);

  bool write_word(
    uint8_t id,
    uint8_t address,
    uint16_t value);

  bool read_word(
    uint8_t id,
    uint8_t address,
    uint16_t & value);

  bool enable_torque(uint8_t id);
  bool disable_torque(uint8_t id);

  int read_motor_tick(std::size_t index);

  bool write_motor_tick(
    std::size_t index,
    int tick);

  int rad_to_tick(
    double rad,
    std::size_t index) const;

  double tick_to_rad(
    int tick,
    std::size_t index) const;

  int joint_name_to_motor_id(
    const std::string & joint_name) const;

  void torque_callback(
    const std_msgs::msg::Bool::SharedPtr msg);

  void set_all_torque(bool enable);
};

}  // namespace so_arm101_driver

#endif  // SO_ARM101_DRIVER__SO_ARM101_HARDWARE_HPP_