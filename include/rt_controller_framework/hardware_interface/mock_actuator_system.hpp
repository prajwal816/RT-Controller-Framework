/**
 * @file mock_actuator_system.hpp
 * @brief Mock actuator hardware interface for ros2_control.
 *
 * Implements hardware_interface::SystemInterface to simulate a multi-joint
 * robotic actuator with position, velocity, and effort command/state
 * interfaces. Uses simple Euler integration for dynamics simulation.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#ifndef RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_
#define RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_

#include <string>
#include <vector>

#include "hardware_interface/system_interface.hpp"
#include "hardware_interface/handle.hpp"
#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_return_values.hpp"
#include "rclcpp/macros.hpp"
#include "rclcpp_lifecycle/state.hpp"

namespace rt_controller_framework {
namespace hardware_interface {

/**
 * @brief Mock actuator system implementing ros2_control SystemInterface.
 *
 * Provides configurable joints with:
 *   - State interfaces: position, velocity, effort
 *   - Command interfaces: position, velocity, effort
 *
 * Actuator dynamics are simulated via simple Euler integration:
 *   velocity += (effort_command / inertia) * dt
 *   position += velocity * dt
 *
 * All memory is pre-allocated during on_init() / on_configure().
 */
class MockActuatorSystem : public ::hardware_interface::SystemInterface {
 public:
  RCLCPP_SHARED_PTR_DEFINITIONS(MockActuatorSystem)

  /**
   * @brief Initialize the hardware interface from URDF info.
   */
  ::hardware_interface::CallbackReturn on_init(
      const ::hardware_interface::HardwareInfo& info) override;

  /**
   * @brief Configure the hardware (pre-allocate state/command vectors).
   */
  ::hardware_interface::CallbackReturn on_configure(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Activate the hardware (reset states).
   */
  ::hardware_interface::CallbackReturn on_activate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Deactivate the hardware.
   */
  ::hardware_interface::CallbackReturn on_deactivate(
      const rclcpp_lifecycle::State& previous_state) override;

  /**
   * @brief Export state interfaces for controller_manager.
   */
  std::vector<::hardware_interface::StateInterface>
  export_state_interfaces() override;

  /**
   * @brief Export command interfaces for controller_manager.
   */
  std::vector<::hardware_interface::CommandInterface>
  export_command_interfaces() override;

  /**
   * @brief Read sensor states (simulate sensor feedback).
   */
  ::hardware_interface::return_type read(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

  /**
   * @brief Write actuator commands (simulate actuator dynamics).
   */
  ::hardware_interface::return_type write(
      const rclcpp::Time& time,
      const rclcpp::Duration& period) override;

 private:
  // Pre-allocated state vectors (no dynamic allocation in read/write)
  std::vector<double> hw_positions_;
  std::vector<double> hw_velocities_;
  std::vector<double> hw_efforts_;

  // Pre-allocated command vectors
  std::vector<double> hw_cmd_positions_;
  std::vector<double> hw_cmd_velocities_;
  std::vector<double> hw_cmd_efforts_;

  // Simulation parameters
  double inertia_{1.0};
  double damping_{0.1};
  double dt_{0.001};  // 1kHz default
};

}  // namespace hardware_interface
}  // namespace rt_controller_framework

#endif  // RT_CONTROLLER_FRAMEWORK__HARDWARE_INTERFACE__MOCK_ACTUATOR_SYSTEM_HPP_
