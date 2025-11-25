#include "so_arm_hardware/so_arm_hardware_interface.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <thread>
#include <vector>

#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rclcpp/rclcpp.hpp"

namespace so_arm_hardware
{

hardware_interface::CallbackReturn SoArmHardwareInterface::on_init(
  const hardware_interface::HardwareInfo & info)
{
  if (
    hardware_interface::SystemInterface::on_init(info) !=
    hardware_interface::CallbackReturn::SUCCESS)
  {
    return hardware_interface::CallbackReturn::ERROR;
  }

  // Get serial port parameter
  serial_port_ = info_.hardware_parameters["port"];
  baud_rate_ = std::stoi(info_.hardware_parameters["baudrate"]);

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmHardwareInterface"),
    "Initializing SO-ARM hardware interface with port: %s, baudrate: %d",
    serial_port_.c_str(), baud_rate_);

  // Initialize storage vectors
  hw_positions_.resize(info_.joints.size(), 0.0);
  hw_velocities_.resize(info_.joints.size(), 0.0);
  hw_commands_.resize(info_.joints.size(), 0.0);
  hw_commands_previous_.resize(info_.joints.size(), 0.0);
  position_offsets_.resize(info_.joints.size(), 2048.0);
  position_scales_.resize(info_.joints.size(), 0.00153398);  // 2*PI/4096

  // Parse servo IDs from URDF parameters
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    const auto & joint = info_.joints[i];

    // Get servo ID from joint parameters
    if (joint.parameters.find("servo_id") != joint.parameters.end())
    {
      uint8_t servo_id = std::stoi(joint.parameters.at("servo_id"));
      servo_ids_[joint.name] = servo_id;

      RCLCPP_INFO(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Joint '%s' mapped to servo ID %d", joint.name.c_str(), servo_id);
    }
    else
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Joint '%s' missing 'servo_id' parameter!", joint.name.c_str());
      return hardware_interface::CallbackReturn::ERROR;
    }

    // Get position offset if specified
    if (joint.parameters.find("position_offset") != joint.parameters.end())
    {
      position_offsets_[i] = std::stod(joint.parameters.at("position_offset"));
    }

    // Get position scale if specified
    if (joint.parameters.find("position_scale") != joint.parameters.end())
    {
      position_scales_[i] = std::stod(joint.parameters.at("position_scale"));
    }

    // Verify command and state interfaces
    if (joint.command_interfaces.size() != 1)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Joint '%s' has %zu command interfaces. Expected 1.",
        joint.name.c_str(), joint.command_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.command_interfaces[0].name != hardware_interface::HW_IF_POSITION)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Joint '%s' has '%s' command interface. Expected '%s'.",
        joint.name.c_str(), joint.command_interfaces[0].name.c_str(),
        hardware_interface::HW_IF_POSITION);
      return hardware_interface::CallbackReturn::ERROR;
    }

    if (joint.state_interfaces.size() != 2)
    {
      RCLCPP_FATAL(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Joint '%s' has %zu state interfaces. Expected 2.",
        joint.name.c_str(), joint.state_interfaces.size());
      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Get safety parameters
  if (info_.hardware_parameters.find("max_temperature") != info_.hardware_parameters.end())
  {
    max_temperature_ = std::stod(info_.hardware_parameters["max_temperature"]);
  }
  else
  {
    max_temperature_ = 70.0;  // Default 70°C
  }

  if (info_.hardware_parameters.find("max_current") != info_.hardware_parameters.end())
  {
    max_current_ = std::stoi(info_.hardware_parameters["max_current"]);
  }
  else
  {
    max_current_ = 1000;  // Default 1000mA
  }

  // Get communication parameters
  read_timeout_ms_ = 10;
  write_timeout_ms_ = 5;
  max_retries_ = 3;

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmHardwareInterface"),
    "Hardware interface initialized successfully with %zu joints",
    info_.joints.size());

  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmHardwareInterface::on_configure(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Configuring hardware...");

  if (!initializeServoDriver())
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoArmHardwareInterface"), "Failed to initialize servo driver!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  if (!scanAndVerifyServos())
  {
    RCLCPP_ERROR(rclcpp::get_logger("SoArmHardwareInterface"), "Failed to verify all servos!");
    return hardware_interface::CallbackReturn::ERROR;
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Hardware configured successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmHardwareInterface::on_cleanup(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Cleaning up hardware...");

  if (servo_driver_)
  {
    servo_driver_->close();
    servo_driver_.reset();
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Hardware cleaned up");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmHardwareInterface::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Activating hardware...");

  // Enable torque for all servos
  for (const auto & [joint_name, servo_id] : servo_ids_)
  {
    // First, disable torque to reset servo state (in case it's already enabled)
    RCLCPP_DEBUG(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Resetting servo %d (joint: %s)", servo_id, joint_name.c_str());

    servo_driver_->enableTorque(servo_id, false);
    std::this_thread::sleep_for(std::chrono::milliseconds(50));

    // Now enable torque
    int max_retries = 3;
    bool success = false;

    for (int retry = 0; retry < max_retries; retry++)
    {
      if (retry > 0)
      {
        RCLCPP_WARN(
          rclcpp::get_logger("SoArmHardwareInterface"),
          "Retrying to enable torque for servo %d (attempt %d/%d)",
          servo_id, retry + 1, max_retries);

        // Small delay before retry
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
      }

      int result = servo_driver_->enableTorque(servo_id, true);
      // Note: Ack() returns 1 for success, 0 for failure
      if (result == 1)
      {
        success = true;
        RCLCPP_INFO(
          rclcpp::get_logger("SoArmHardwareInterface"),
          "Enabled torque for servo %d (joint: %s)", servo_id, joint_name.c_str());
        break;
      }
      else
      {
        RCLCPP_DEBUG(
          rclcpp::get_logger("SoArmHardwareInterface"),
          "Enable torque returned: %d for servo %d (communication %s)",
          result, servo_id, (result == 0 ? "error" : "unknown"));
      }
    }

    if (!success)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Failed to enable torque for servo %d (joint: %s) after %d attempts",
        servo_id, joint_name.c_str(), max_retries);

      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Possible causes: 1) Servo is locked 2) Communication error 3) Servo power issue");

      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Try running: cd ~/FTServo_Linux/examples/STSServo && ./demo_basic /dev/ttyACM0");

      return hardware_interface::CallbackReturn::ERROR;
    }
  }

  // Read current positions as initial commands
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    uint8_t servo_id = servo_ids_[info_.joints[i].name];
    int pos = servo_driver_->readPosition(servo_id);
    if (pos >= 0)
    {
      hw_positions_[i] = servoUnitsToRadians(pos, i);
      hw_commands_[i] = hw_positions_[i];
      hw_commands_previous_[i] = hw_positions_[i];
    }
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Hardware activated successfully");
  return hardware_interface::CallbackReturn::SUCCESS;
}

hardware_interface::CallbackReturn SoArmHardwareInterface::on_deactivate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Deactivating hardware...");

  // Disable torque for all servos
  for (const auto & [joint_name, servo_id] : servo_ids_)
  {
    servo_driver_->enableTorque(servo_id, false);
    RCLCPP_INFO(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Disabled torque for servo %d (joint: %s)", servo_id, joint_name.c_str());
  }

  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Hardware deactivated");
  return hardware_interface::CallbackReturn::SUCCESS;
}

std::vector<hardware_interface::StateInterface>
SoArmHardwareInterface::export_state_interfaces()
{
  std::vector<hardware_interface::StateInterface> state_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));

    state_interfaces.emplace_back(
      hardware_interface::StateInterface(
        info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
  }

  return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
SoArmHardwareInterface::export_command_interfaces()
{
  std::vector<hardware_interface::CommandInterface> command_interfaces;

  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    command_interfaces.emplace_back(
      hardware_interface::CommandInterface(
        info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_[i]));
  }

  return command_interfaces;
}

hardware_interface::return_type SoArmHardwareInterface::read(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Read positions and velocities from all servos
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    uint8_t servo_id = servo_ids_[info_.joints[i].name];

    // Read position
    int pos = servo_driver_->readPosition(servo_id);
    if (pos >= 0)
    {
      hw_positions_[i] = servoUnitsToRadians(pos, i);
    }

    // Read velocity (speed)
    int spd = servo_driver_->readSpeed(servo_id);
    if (spd >= 0)
    {
      // Convert servo speed units to rad/s
      // Assuming speed unit is in some servo-specific format
      hw_velocities_[i] = spd * 0.001;  // Placeholder conversion
    }
  }

  return hardware_interface::return_type::OK;
}

hardware_interface::return_type SoArmHardwareInterface::write(
  const rclcpp::Time & /*time*/, const rclcpp::Duration & /*period*/)
{
  // Prepare data for synchronous write
  std::vector<uint8_t> ids;
  std::vector<int16_t> positions;
  std::vector<uint16_t> speeds;
  std::vector<uint8_t> accelerations;

  // Collect changed commands
  for (size_t i = 0; i < info_.joints.size(); i++)
  {
    // Check if command has changed significantly (avoid unnecessary writes)
    if (std::abs(hw_commands_[i] - hw_commands_previous_[i]) > 0.001)  // 1 mrad threshold
    {
      uint8_t servo_id = servo_ids_[info_.joints[i].name];
      int16_t target_pos = radiansToServoUnits(hw_commands_[i], i);

      ids.push_back(servo_id);
      positions.push_back(target_pos);
      speeds.push_back(2400);  // Default speed, can be made configurable
      accelerations.push_back(50);  // Default acceleration

      hw_commands_previous_[i] = hw_commands_[i];
    }
  }

  // Execute synchronous write if there are any changes
  if (!ids.empty())
  {
    servo_driver_->syncWritePosition(
      ids.data(), positions.data(), speeds.data(), accelerations.data(), ids.size());
  }

  return hardware_interface::return_type::OK;
}

// Private helper methods

bool SoArmHardwareInterface::initializeServoDriver()
{
  servo_driver_ = std::make_unique<STSServo>();

  if (!servo_driver_->init(serial_port_.c_str(), baud_rate_))
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Failed to initialize servo driver on port %s with baudrate %d",
      serial_port_.c_str(), baud_rate_);
    return false;
  }

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmHardwareInterface"),
    "Servo driver initialized successfully");

  return true;
}

bool SoArmHardwareInterface::scanAndVerifyServos()
{
  RCLCPP_INFO(rclcpp::get_logger("SoArmHardwareInterface"), "Scanning for servos...");

  // Give servo driver time to settle after initialization
  std::this_thread::sleep_for(std::chrono::milliseconds(200));

  std::vector<uint8_t> found_servos = servo_driver_->scanServos(1, 20);

  RCLCPP_INFO(
    rclcpp::get_logger("SoArmHardwareInterface"),
    "Found %zu servos on the bus", found_servos.size());

  // Verify all required servos are present
  for (const auto & [joint_name, servo_id] : servo_ids_)
  {
    bool found = false;
    for (uint8_t id : found_servos)
    {
      if (id == servo_id)
      {
        found = true;
        break;
      }
    }

    if (!found)
    {
      RCLCPP_ERROR(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Servo ID %d for joint '%s' not found on the bus!",
        servo_id, joint_name.c_str());
      return false;
    }

    RCLCPP_INFO(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Verified servo ID %d for joint '%s'", servo_id, joint_name.c_str());

    // Check servo health
    if (!checkServoHealth(servo_id))
    {
      RCLCPP_WARN(
        rclcpp::get_logger("SoArmHardwareInterface"),
        "Servo %d health check warning", servo_id);
    }
  }

  return true;
}

int16_t SoArmHardwareInterface::radiansToServoUnits(double radians, size_t joint_index)
{
  // Convert radians to servo units (0-4095)
  // servo_units = offset + (radians / scale)
  int16_t units = static_cast<int16_t>(
    position_offsets_[joint_index] + (radians / position_scales_[joint_index]));

  // Clamp to valid range
  if (units < 0) units = 0;
  if (units > 4095) units = 4095;

  return units;
}

double SoArmHardwareInterface::servoUnitsToRadians(int16_t units, size_t joint_index)
{
  // Convert servo units to radians
  // radians = (units - offset) * scale
  return (units - position_offsets_[joint_index]) * position_scales_[joint_index];
}

bool SoArmHardwareInterface::checkServoHealth(uint8_t servo_id)
{
  ServoInfo info;
  if (!servo_driver_->readServoInfo(servo_id, info))
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Failed to read servo %d info", servo_id);
    return false;
  }

  // Check temperature
  if (info.temperature > max_temperature_)
  {
    RCLCPP_ERROR(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Servo %d temperature too high: %d°C (max: %.1f°C)",
      servo_id, info.temperature, max_temperature_);
    return false;
  }

  // Check current
  if (std::abs(info.current) > max_current_)
  {
    RCLCPP_WARN(
      rclcpp::get_logger("SoArmHardwareInterface"),
      "Servo %d current high: %dmA (max: %d mA)",
      servo_id, info.current, max_current_);
  }

  RCLCPP_DEBUG(
    rclcpp::get_logger("SoArmHardwareInterface"),
    "Servo %d: pos=%d, temp=%d°C, voltage=%.1fV, current=%dmA",
    servo_id, info.position, info.temperature, info.voltage / 10.0, info.current);

  return true;
}

}  // namespace so_arm_hardware

#include "pluginlib/class_list_macros.hpp"
PLUGINLIB_EXPORT_CLASS(
  so_arm_hardware::SoArmHardwareInterface, hardware_interface::SystemInterface)
