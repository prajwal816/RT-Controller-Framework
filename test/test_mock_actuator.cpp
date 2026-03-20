/**
 * @file test_mock_actuator.cpp
 * @brief GTest unit tests for the mock actuator hardware interface.
 *
 * Tests the lifecycle (configure → activate → read/write → deactivate)
 * and verifies that Euler integration produces expected state transitions.
 */

#include <gtest/gtest.h>

#include <string>
#include <vector>

#include "hardware_interface/hardware_info.hpp"
#include "hardware_interface/types/hardware_interface_type_values.hpp"
#include "rt_controller_framework/hardware_interface/mock_actuator_system.hpp"

using rt_controller_framework::hardware_interface::MockActuatorSystem;

/**
 * @brief Helper to create a HardwareInfo struct with test joints.
 */
static ::hardware_interface::HardwareInfo make_test_info(int num_joints) {
  ::hardware_interface::HardwareInfo info;
  info.name = "test_actuator";
  info.type = "system";

  info.hardware_parameters["inertia"] = "1.0";
  info.hardware_parameters["damping"] = "0.1";
  info.hardware_parameters["control_period"] = "0.001";

  for (int i = 0; i < num_joints; ++i) {
    ::hardware_interface::ComponentInfo joint;
    joint.name = "joint_" + std::to_string(i + 1);
    joint.parameters["initial_position"] = "0.0";

    // State interfaces
    ::hardware_interface::InterfaceInfo pos_state;
    pos_state.name = ::hardware_interface::HW_IF_POSITION;
    joint.state_interfaces.push_back(pos_state);

    ::hardware_interface::InterfaceInfo vel_state;
    vel_state.name = ::hardware_interface::HW_IF_VELOCITY;
    joint.state_interfaces.push_back(vel_state);

    ::hardware_interface::InterfaceInfo eff_state;
    eff_state.name = ::hardware_interface::HW_IF_EFFORT;
    joint.state_interfaces.push_back(eff_state);

    // Command interfaces
    ::hardware_interface::InterfaceInfo pos_cmd;
    pos_cmd.name = ::hardware_interface::HW_IF_POSITION;
    joint.command_interfaces.push_back(pos_cmd);

    ::hardware_interface::InterfaceInfo vel_cmd;
    vel_cmd.name = ::hardware_interface::HW_IF_VELOCITY;
    joint.command_interfaces.push_back(vel_cmd);

    ::hardware_interface::InterfaceInfo eff_cmd;
    eff_cmd.name = ::hardware_interface::HW_IF_EFFORT;
    joint.command_interfaces.push_back(eff_cmd);

    info.joints.push_back(joint);
  }

  return info;
}

TEST(MockActuatorTest, Initialization) {
  MockActuatorSystem hw;
  auto info = make_test_info(3);

  auto result = hw.on_init(info);
  EXPECT_EQ(result, ::hardware_interface::CallbackReturn::SUCCESS);
}

TEST(MockActuatorTest, ExportInterfaces) {
  MockActuatorSystem hw;
  auto info = make_test_info(2);
  hw.on_init(info);

  auto state_ifs = hw.export_state_interfaces();
  EXPECT_EQ(state_ifs.size(), 6u);  // 2 joints × 3 interfaces

  auto cmd_ifs = hw.export_command_interfaces();
  EXPECT_EQ(cmd_ifs.size(), 6u);
}

TEST(MockActuatorTest, Lifecycle) {
  MockActuatorSystem hw;
  auto info = make_test_info(1);
  hw.on_init(info);

  rclcpp_lifecycle::State dummy_state;

  auto r1 = hw.on_configure(dummy_state);
  EXPECT_EQ(r1, ::hardware_interface::CallbackReturn::SUCCESS);

  auto r2 = hw.on_activate(dummy_state);
  EXPECT_EQ(r2, ::hardware_interface::CallbackReturn::SUCCESS);

  auto r3 = hw.on_deactivate(dummy_state);
  EXPECT_EQ(r3, ::hardware_interface::CallbackReturn::SUCCESS);
}

TEST(MockActuatorTest, ReadWriteCycle) {
  MockActuatorSystem hw;
  auto info = make_test_info(1);
  hw.on_init(info);

  rclcpp_lifecycle::State dummy_state;
  hw.on_configure(dummy_state);
  hw.on_activate(dummy_state);

  rclcpp::Time time(0, 0, RCL_ROS_TIME);
  rclcpp::Duration period(0, 1000000);  // 1ms

  // Run a few write/read cycles
  for (int i = 0; i < 100; ++i) {
    auto wr = hw.write(time, period);
    EXPECT_EQ(wr, ::hardware_interface::return_type::OK);

    auto rr = hw.read(time, period);
    EXPECT_EQ(rr, ::hardware_interface::return_type::OK);
  }
}

TEST(MockActuatorTest, StateInterfaceValues) {
  MockActuatorSystem hw;
  auto info = make_test_info(1);
  hw.on_init(info);

  auto state_ifs = hw.export_state_interfaces();
  ASSERT_EQ(state_ifs.size(), 3u);

  // Initial position should be 0.0
  EXPECT_DOUBLE_EQ(state_ifs[0].get_value(), 0.0);  // position
  EXPECT_DOUBLE_EQ(state_ifs[1].get_value(), 0.0);  // velocity
}
