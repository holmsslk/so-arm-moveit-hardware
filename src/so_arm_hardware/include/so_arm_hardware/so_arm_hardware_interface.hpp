#ifndef SO_ARM_HARDWARE__SO_ARM_HARDWARE_INTERFACE_HPP_
#define SO_ARM_HARDWARE__SO_ARM_HARDWARE_INTERFACE_HPP_

#include <map>
#include <memory>
#include <string>
#include <vector>

#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp"
#include "rclcpp_lifecycle/state.hpp"

#include "STSServo.h"

namespace so_arm_hardware
{

class SoArmHardwareInterface : public hardware_interface::SystemInterface
{
public:
  RCLCPP_SHARED_PTR_DEFINITIONS(SoArmHardwareInterface)

  /**
   * @brief Initialize the hardware interface from URDF hardware info
   */
  hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;

  /**
   * @brief Configure the hardware interface
   */
  hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Cleanup the hardware interface
   */
  hardware_interface::CallbackReturn on_cleanup(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Activate the hardware interface (enable torque)
   */
  hardware_interface::CallbackReturn on_activate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Deactivate the hardware interface (disable torque)
   */
  hardware_interface::CallbackReturn on_deactivate(
    const rclcpp_lifecycle::State & previous_state) override;

  /**
   * @brief Export state interfaces for joint positions and velocities
   */
  std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

  /**
   * @brief Export command interfaces for joint positions
   */
  std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

  /**
   * @brief Read current joint states from hardware
   */
  hardware_interface::return_type read(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

  /**
   * @brief Write joint commands to hardware
   */
  hardware_interface::return_type write(
    const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
  // Feetech servo driver
  std::unique_ptr<STSServo> servo_driver_;

  // Hardware parameters
  std::string serial_port_;
  int baud_rate_;

  // Servo ID mapping: joint_name -> servo_id
  std::map<std::string, uint8_t> servo_ids_;

  // Joint state storage
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_commands_;

  // Previous commands to detect changes
  std::vector<double> hw_commands_previous_;

  // Position conversion factors (servo units to radians)
  std::vector<double> position_offsets_;    // Center position offset
  std::vector<double> position_scales_;     // Scale factor (rad per servo unit)

  // Safety parameters
  double max_temperature_;
  int max_current_;

  // Communication parameters
  int read_timeout_ms_;
  int write_timeout_ms_;
  int max_retries_;

  // Helper methods
  bool initializeServoDriver();
  bool scanAndVerifyServos();
  int16_t radiansToServoUnits(double radians, size_t joint_index);
  double servoUnitsToRadians(int16_t units, size_t joint_index);
  bool checkServoHealth(uint8_t servo_id);
};

}  // namespace so_arm_hardware

#endif  // SO_ARM_HARDWARE__SO_ARM_HARDWARE_INTERFACE_HPP_
