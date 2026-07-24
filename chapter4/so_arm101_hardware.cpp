#include "so_arm101_driver/so_arm101_hardware.hpp"

#include <algorithm>
#include <cerrno>
#include <chrono>
#include <cmath>
#include <cstring>
#include <fcntl.h>
#include <functional>
#include <termios.h>
#include <unistd.h>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "pluginlib/class_list_macros.hpp"
#include "rclcpp/executors/single_threaded_executor.hpp"

namespace so_arm101_driver
{

static constexpr uint8_t INST_READ = 0x02;
static constexpr uint8_t INST_WRITE = 0x03;

// STS3215 / SMS_STS Control Table
static constexpr uint8_t ADDR_TORQUE_ENABLE = 40;
static constexpr uint8_t ADDR_GOAL_ACCELERATION = 41;
static constexpr uint8_t ADDR_GOAL_POSITION_L = 42;
static constexpr uint8_t ADDR_GOAL_SPEED_L = 46;
static constexpr uint8_t ADDR_PRESENT_POSITION_L = 56;

static constexpr uint8_t TORQUE_OFF = 0;
static constexpr uint8_t TORQUE_ON = 1;

static constexpr double PI = 3.14159265358979323846;


hardware_interface::CallbackReturn SOArm101Hardware::on_init(
  const hardware_interface::HardwareComponentInterfaceParams & params)
{
  if (
    hardware_interface::SystemInterface::on_init(params) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (info_.hardware_parameters.count("port") > 0) {
    port_ = info_.hardware_parameters["port"];
  }

  if (info_.hardware_parameters.count("baudrate") > 0) {
    baudrate_ = std::stoi(
      info_.hardware_parameters["baudrate"]);
  }

  joint_names_.clear();
  motor_ids_.clear();

  for (const auto & joint : info_.joints)
  {
    const int motor_id =
      joint_name_to_motor_id(joint.name);

    if (motor_id < 0)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOArm101Hardware"),
        "Unknown joint name: %s",
        joint.name.c_str());

      return hardware_interface::CallbackReturn::ERROR;
    }

    joint_names_.push_back(joint.name);
    motor_ids_.push_back(motor_id);
  }

  const std::size_t joint_count =
    joint_names_.size();

  hw_positions_.assign(joint_count, 0.0);
  hw_velocities_.assign(joint_count, 0.0);
  hw_commands_.assign(joint_count, 0.0);
  prev_positions_.assign(joint_count, 0.0);
  last_command_ticks_.assign(joint_count, -1);

  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "Initialized. port=%s baudrate=%d joints=%zu",
    port_.c_str(),
    baudrate_,
    joint_count);

  return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
SOArm101Hardware::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> interfaces;

  for (std::size_t i = 0;
       i < joint_names_.size();
       ++i)
  {
    interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_positions_[i]);

    interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_VELOCITY,
      &hw_velocities_[i]);
  }

  return interfaces;
}


std::vector<hardware_interface::CommandInterface>
SOArm101Hardware::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> interfaces;

  for (std::size_t i = 0;
       i < joint_names_.size();
       ++i)
  {
    interfaces.emplace_back(
      joint_names_[i],
      hardware_interface::HW_IF_POSITION,
      &hw_commands_[i]);
  }

  return interfaces;
}


hardware_interface::CallbackReturn SOArm101Hardware::on_activate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "Activating SO-ARM101 hardware...");

  if (!open_port()) {
    return hardware_interface::CallbackReturn::ERROR;
  }

  connected_.store(true);
  torque_enabled_.store(false);

  for (std::size_t i = 0;
       i < motor_ids_.size();
       ++i)
  {
    const uint8_t id =
      static_cast<uint8_t>(motor_ids_[i]);

    // 초기화 중 갑작스러운 동작을 막기 위해
    // 먼저 Torque를 해제합니다.
    disable_torque(id);

    // 기본 가속도와 속도를 설정합니다.
    write_byte(
      id,
      ADDR_GOAL_ACCELERATION,
      20);

    write_word(
      id,
      ADDR_GOAL_SPEED_L,
      800);

    // 현재 위치를 초기 명령값으로 사용합니다.
    const int tick = read_motor_tick(i);
    const double rad = tick_to_rad(tick, i);

    hw_positions_[i] = rad;
    prev_positions_[i] = rad;
    hw_velocities_[i] = 0.0;
    hw_commands_[i] = rad;
    last_command_ticks_[i] = tick;
  }

  for (const int motor_id : motor_ids_)
  {
    if (!enable_torque(
        static_cast<uint8_t>(motor_id)))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOArm101Hardware"),
        "Failed to enable torque. motor_id=%d",
        motor_id);

      connected_.store(false);
      close_port();

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  torque_enabled_.store(true);

  torque_node_ = rclcpp::Node::make_shared(
    "so_arm101_torque_interface");

  torque_sub_ =
    torque_node_->create_subscription<std_msgs::msg::Bool>(
    "/so_arm101/torque_enable",
    10,
    std::bind(
      &SOArm101Hardware::torque_callback,
      this,
      std::placeholders::_1));

  torque_spin_running_.store(true);

  torque_spin_thread_ = std::thread(
    [this]()
    {
      rclcpp::executors::SingleThreadedExecutor executor;

      executor.add_node(torque_node_);

      while (
        rclcpp::ok() &&
        torque_spin_running_.load())
      {
        executor.spin_some();

        std::this_thread::sleep_for(
          std::chrono::milliseconds(10));
      }

      executor.remove_node(torque_node_);
    });

  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "SO-ARM101 hardware activated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::CallbackReturn SOArm101Hardware::on_deactivate(
  const rclcpp_lifecycle::State &)
{
  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "Deactivating SO-ARM101 hardware...");

  torque_spin_running_.store(false);

  if (torque_spin_thread_.joinable()) {
    torque_spin_thread_.join();
  }

  torque_sub_.reset();
  torque_node_.reset();

  torque_enabled_.store(false);

  if (connected_.load())
  {
    for (const int motor_id : motor_ids_) {
      disable_torque(
        static_cast<uint8_t>(motor_id));
    }
  }

  connected_.store(false);
  close_port();

  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "SO-ARM101 hardware deactivated.");

  return hardware_interface::CallbackReturn::SUCCESS;
}


hardware_interface::return_type SOArm101Hardware::read(
  const rclcpp::Time &,
  const rclcpp::Duration & period)
{
  if (!connected_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  const double dt = period.seconds();

  for (std::size_t i = 0;
       i < joint_names_.size();
       ++i)
  {
    const int tick = read_motor_tick(i);
    const double rad = tick_to_rad(tick, i);

    std::lock_guard<std::mutex> lock(state_mutex_);

    hw_positions_[i] = rad;

    if (dt > 0.0) {
      hw_velocities_[i] =
        (rad - prev_positions_[i]) / dt;
    } else {
      hw_velocities_[i] = 0.0;
    }

    prev_positions_[i] = rad;
  }

  return hardware_interface::return_type::OK;
}


hardware_interface::return_type SOArm101Hardware::write(
  const rclcpp::Time &,
  const rclcpp::Duration &)
{
  if (!connected_.load()) {
    return hardware_interface::return_type::ERROR;
  }

  if (!torque_enabled_.load()) {
    return hardware_interface::return_type::OK;
  }

  for (std::size_t i = 0;
       i < joint_names_.size();
       ++i)
  {
    double command_rad = 0.0;

    {
      std::lock_guard<std::mutex> lock(
        state_mutex_);

      command_rad = hw_commands_[i];
    }

    const int tick =
      rad_to_tick(command_rad, i);

    // 3 Tick 미만의 작은 변화는 전송하지 않습니다.
    if (
      std::abs(
        tick - last_command_ticks_[i]) < 3)
    {
      continue;
    }

    if (!write_motor_tick(i, tick))
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SOArm101Hardware"),
        "Failed to write position. joint=%s",
        joint_names_[i].c_str());

      return hardware_interface::return_type::ERROR;
    }

    last_command_ticks_[i] = tick;
  }

  return hardware_interface::return_type::OK;
}


bool SOArm101Hardware::open_port()
{
  fd_ = open(
    port_.c_str(),
    O_RDWR | O_NOCTTY | O_SYNC);

  if (fd_ < 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SOArm101Hardware"),
      "Failed to open port %s: %s",
      port_.c_str(),
      std::strerror(errno));

    return false;
  }

  termios tty{};

  if (tcgetattr(fd_, &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SOArm101Hardware"),
      "tcgetattr failed: %s",
      std::strerror(errno));

    close(fd_);
    fd_ = -1;

    return false;
  }

  cfmakeraw(&tty);

  speed_t speed = B1000000;

  if (baudrate_ == 57600) {
    speed = B57600;
  } else if (baudrate_ == 115200) {
    speed = B115200;
  } else if (baudrate_ == 1000000) {
    speed = B1000000;
  } else {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Unsupported baudrate %d. Using 1000000.",
      baudrate_);
  }

  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);

  tty.c_cflag |= CLOCAL | CREAD;
  tty.c_cflag &= ~CSIZE;
  tty.c_cflag |= CS8;
  tty.c_cflag &= ~PARENB;
  tty.c_cflag &= ~CSTOPB;
  tty.c_cflag &= ~CRTSCTS;

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 2;

  if (tcsetattr(
      fd_,
      TCSANOW,
      &tty) != 0)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SOArm101Hardware"),
      "tcsetattr failed: %s",
      std::strerror(errno));

    close(fd_);
    fd_ = -1;

    return false;
  }

  tcflush(fd_, TCIOFLUSH);

  RCLCPP_INFO(
    rclcpp::get_logger("SOArm101Hardware"),
    "Opened serial port: %s",
    port_.c_str());

  return true;
}


void SOArm101Hardware::close_port()
{
  std::lock_guard<std::mutex> lock(
    bus_mutex_);

  if (fd_ >= 0)
  {
    close(fd_);
    fd_ = -1;
  }
}


bool SOArm101Hardware::send_packet(
  uint8_t id,
  uint8_t instruction,
  const std::vector<uint8_t> & params)
{
  if (fd_ < 0) {
    return false;
  }

  std::vector<uint8_t> packet;

  packet.push_back(0xFF);
  packet.push_back(0xFF);
  packet.push_back(id);

  const uint8_t length =
    static_cast<uint8_t>(
      params.size() + 2);

  packet.push_back(length);
  packet.push_back(instruction);

  uint16_t sum =
    id + length + instruction;

  for (const uint8_t param : params)
  {
    packet.push_back(param);
    sum += param;
  }

  packet.push_back(
    static_cast<uint8_t>(~sum));

  tcflush(fd_, TCIFLUSH);

  const ssize_t written = ::write(
    fd_,
    packet.data(),
    packet.size());

  if (
    written !=
    static_cast<ssize_t>(packet.size()))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SOArm101Hardware"),
      "Serial packet write failed.");

    return false;
  }

  tcdrain(fd_);

  return true;
}


bool SOArm101Hardware::read_status_packet(
  uint8_t expected_id,
  std::vector<uint8_t> & params)
{
  params.clear();

  uint8_t data = 0;
  bool header_found = false;

  for (int i = 0; i < 100; ++i)
  {
    if (::read(fd_, &data, 1) != 1) {
      continue;
    }

    if (data != 0xFF) {
      continue;
    }

    if (::read(fd_, &data, 1) != 1) {
      continue;
    }

    if (data == 0xFF)
    {
      header_found = true;
      break;
    }
  }

  if (!header_found) {
    return false;
  }

  uint8_t id = 0;
  uint8_t length = 0;
  uint8_t error = 0;

  if (::read(fd_, &id, 1) != 1) {
    return false;
  }

  if (::read(fd_, &length, 1) != 1) {
    return false;
  }

  if (::read(fd_, &error, 1) != 1) {
    return false;
  }

  if (id != expected_id)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Unexpected servo id. expected=%d received=%d",
      expected_id,
      id);

    return false;
  }

  if (length < 2) {
    return false;
  }

  const int parameter_length =
    static_cast<int>(length) - 2;

  params.resize(parameter_length);

  for (int i = 0;
       i < parameter_length;
       ++i)
  {
    if (::read(
        fd_,
        &params[i],
        1) != 1)
    {
      return false;
    }
  }

  uint8_t checksum = 0;

  if (::read(fd_, &checksum, 1) != 1) {
    return false;
  }

  uint16_t sum = id + length + error;

  for (const uint8_t param : params) {
    sum += param;
  }

  const uint8_t calculated_checksum =
    static_cast<uint8_t>(~sum);

  if (checksum != calculated_checksum)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Checksum error.");

    return false;
  }

  if (error != 0)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Servo status error. id=%d error=%d",
      id,
      error);
  }

  return true;
}


bool SOArm101Hardware::write_byte(
  uint8_t id,
  uint8_t address,
  uint8_t value)
{
  std::lock_guard<std::mutex> lock(
    bus_mutex_);

  return send_packet(
    id,
    INST_WRITE,
    {address, value});
}


bool SOArm101Hardware::write_word(
  uint8_t id,
  uint8_t address,
  uint16_t value)
{
  std::lock_guard<std::mutex> lock(
    bus_mutex_);

  return send_packet(
    id,
    INST_WRITE,
    {
      address,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>(
        (value >> 8) & 0xFF)
    });
}


bool SOArm101Hardware::read_word(
  uint8_t id,
  uint8_t address,
  uint16_t & value)
{
  std::lock_guard<std::mutex> lock(
    bus_mutex_);

  if (!send_packet(
      id,
      INST_READ,
      {address, 2}))
  {
    return false;
  }

  std::vector<uint8_t> params;

  if (!read_status_packet(id, params)) {
    return false;
  }

  if (params.size() < 2) {
    return false;
  }

  value = static_cast<uint16_t>(
    params[0] |
    (
      static_cast<uint16_t>(params[1])
      << 8
    )
  );

  return true;
}


bool SOArm101Hardware::enable_torque(
  uint8_t id)
{
  return write_byte(
    id,
    ADDR_TORQUE_ENABLE,
    TORQUE_ON);
}


bool SOArm101Hardware::disable_torque(
  uint8_t id)
{
  return write_byte(
    id,
    ADDR_TORQUE_ENABLE,
    TORQUE_OFF);
}


int SOArm101Hardware::read_motor_tick(
  std::size_t index)
{
  uint16_t value = 2048;

  const uint8_t id =
    static_cast<uint8_t>(
      motor_ids_[index]);

  if (!read_word(
      id,
      ADDR_PRESENT_POSITION_L,
      value))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Failed to read position. joint=%s id=%d",
      joint_names_[index].c_str(),
      id);

    if (last_command_ticks_[index] >= 0) {
      return last_command_ticks_[index];
    }
  }

  return static_cast<int>(value);
}


bool SOArm101Hardware::write_motor_tick(
  std::size_t index,
  int tick)
{
  tick = std::clamp(
    tick,
    0,
    4095);

  const uint8_t id =
    static_cast<uint8_t>(
      motor_ids_[index]);

  return write_word(
    id,
    ADDR_GOAL_POSITION_L,
    static_cast<uint16_t>(tick));
}


double SOArm101Hardware::tick_to_rad(
  int tick,
  std::size_t) const
{
  const double center_tick = 2048.0;

  const double tick_per_rad =
    4096.0 / (2.0 * PI);

  return
    (static_cast<double>(tick) - center_tick) /
    tick_per_rad;
}


int SOArm101Hardware::rad_to_tick(
  double rad,
  std::size_t) const
{
  const double center_tick = 2048.0;

  const double tick_per_rad =
    4096.0 / (2.0 * PI);

  return static_cast<int>(
    std::round(
      center_tick +
      rad * tick_per_rad));
}


int SOArm101Hardware::joint_name_to_motor_id(
  const std::string & joint_name) const
{
  if (joint_name == "shoulder_pan") {
    return 1;
  }

  if (joint_name == "shoulder_lift") {
    return 2;
  }

  if (joint_name == "elbow_flex") {
    return 3;
  }

  if (joint_name == "wrist_flex") {
    return 4;
  }

  if (joint_name == "wrist_roll") {
    return 5;
  }

  if (joint_name == "gripper") {
    return 6;
  }

  return -1;
}


void SOArm101Hardware::torque_callback(
  const std_msgs::msg::Bool::SharedPtr msg)
{
  set_all_torque(msg->data);
}


void SOArm101Hardware::set_all_torque(
  bool enable)
{
  if (!connected_.load())
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SOArm101Hardware"),
      "Torque command ignored. Hardware is not connected.");

    return;
  }

  if (enable)
  {
    for (std::size_t i = 0;
         i < motor_ids_.size();
         ++i)
    {
      const int tick = read_motor_tick(i);
      const double rad = tick_to_rad(tick, i);

      std::lock_guard<std::mutex> lock(
        state_mutex_);

      hw_positions_[i] = rad;
      prev_positions_[i] = rad;
      hw_velocities_[i] = 0.0;
      hw_commands_[i] = rad;
      last_command_ticks_[i] = tick;
    }

    for (const int motor_id : motor_ids_) {
      enable_torque(
        static_cast<uint8_t>(motor_id));
    }

    torque_enabled_.store(true);

    RCLCPP_INFO(
      rclcpp::get_logger("SOArm101Hardware"),
      "Torque enabled from topic.");
  }
  else
  {
    torque_enabled_.store(false);

    for (const int motor_id : motor_ids_) {
      disable_torque(
        static_cast<uint8_t>(motor_id));
    }

    RCLCPP_INFO(
      rclcpp::get_logger("SOArm101Hardware"),
      "Torque disabled from topic.");
  }
}

}  // namespace so_arm101_driver


PLUGINLIB_EXPORT_CLASS(
  so_arm101_driver::SOArm101Hardware,
  hardware_interface::SystemInterface
)