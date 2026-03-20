cmake_minimum_required(VERSION 3.16)
project(rt_controller_framework VERSION 1.0.0 LANGUAGES CXX)

# ─── C++17 Standard ─────────────────────────────────────────────────────────
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -O2)
endif()

# ─── Find Dependencies ──────────────────────────────────────────────────────
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_lifecycle REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(controller_interface REQUIRED)
find_package(hardware_interface REQUIRED)
find_package(controller_manager REQUIRED)
find_package(pluginlib REQUIRED)
find_package(realtime_tools REQUIRED)

set(THIS_PACKAGE_INCLUDE_DEPENDS
  rclcpp
  rclcpp_lifecycle
  std_msgs
  sensor_msgs
  controller_interface
  hardware_interface
  controller_manager
  pluginlib
  realtime_tools
)

# ─── Include Directories ────────────────────────────────────────────────────
include_directories(include)

# ═══════════════════════════════════════════════════════════════════════════════
# Library: rt_realtime_utils
# ═══════════════════════════════════════════════════════════════════════════════
add_library(rt_realtime_utils SHARED
  src/realtime_utils/high_resolution_timer.cpp
  src/realtime_utils/jitter_monitor.cpp
  src/realtime_utils/rt_thread_config.cpp
)
target_include_directories(rt_realtime_utils PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(rt_realtime_utils ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ═══════════════════════════════════════════════════════════════════════════════
# Library: rt_hardware_interface
# ═══════════════════════════════════════════════════════════════════════════════
add_library(rt_hardware_interface SHARED
  src/hardware_interface/mock_actuator_system.cpp
)
target_include_directories(rt_hardware_interface PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rt_hardware_interface rt_realtime_utils)
ament_target_dependencies(rt_hardware_interface ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(hardware_interface rt_hardware_plugin.xml)

# ═══════════════════════════════════════════════════════════════════════════════
# Library: rt_controller
# ═══════════════════════════════════════════════════════════════════════════════
add_library(rt_controller SHARED
  src/controller/rt_controller.cpp
)
target_include_directories(rt_controller PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rt_controller rt_realtime_utils)
ament_target_dependencies(rt_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
pluginlib_export_plugin_description_file(controller_interface rt_controller_plugin.xml)

# ═══════════════════════════════════════════════════════════════════════════════
# Library: rt_executor
# ═══════════════════════════════════════════════════════════════════════════════
add_library(rt_executor SHARED
  src/executor/rt_executor_node.cpp
)
target_include_directories(rt_executor PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
target_link_libraries(rt_executor rt_realtime_utils)
ament_target_dependencies(rt_executor ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ─── Executor Standalone Node ────────────────────────────────────────────────
add_executable(rt_executor_node src/executor/rt_executor_node_main.cpp)
target_link_libraries(rt_executor_node rt_executor rt_realtime_utils)
ament_target_dependencies(rt_executor_node ${THIS_PACKAGE_INCLUDE_DEPENDS})

# ─── Example: RT Loop Demo ──────────────────────────────────────────────────
add_executable(rt_loop_example examples/rt_loop_example.cpp)
target_link_libraries(rt_loop_example rt_realtime_utils)
ament_target_dependencies(rt_loop_example rclcpp)

# ═══════════════════════════════════════════════════════════════════════════════
# Install
# ═══════════════════════════════════════════════════════════════════════════════
install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS
    rt_realtime_utils
    rt_hardware_interface
    rt_controller
    rt_executor
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(
  TARGETS rt_executor_node rt_loop_example
  DESTINATION lib/${PROJECT_NAME}
)

install(
  DIRECTORY config launch urdf
  DESTINATION share/${PROJECT_NAME}
)

install(
  FILES rt_controller_plugin.xml rt_hardware_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

# ═══════════════════════════════════════════════════════════════════════════════
# Testing
# ═══════════════════════════════════════════════════════════════════════════════
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_lock_free_queue test/test_lock_free_queue.cpp)
  target_link_libraries(test_lock_free_queue rt_realtime_utils)
  ament_target_dependencies(test_lock_free_queue ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(test_memory_pool test/test_memory_pool.cpp)
  target_link_libraries(test_memory_pool rt_realtime_utils)
  ament_target_dependencies(test_memory_pool ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(test_jitter_monitor test/test_jitter_monitor.cpp)
  target_link_libraries(test_jitter_monitor rt_realtime_utils)
  ament_target_dependencies(test_jitter_monitor ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(test_high_resolution_timer test/test_high_resolution_timer.cpp)
  target_link_libraries(test_high_resolution_timer rt_realtime_utils)
  ament_target_dependencies(test_high_resolution_timer ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(test_mock_actuator test/test_mock_actuator.cpp)
  target_link_libraries(test_mock_actuator rt_hardware_interface rt_realtime_utils)
  ament_target_dependencies(test_mock_actuator ${THIS_PACKAGE_INCLUDE_DEPENDS})

  ament_add_gtest(test_rt_controller test/test_rt_controller.cpp)
  target_link_libraries(test_rt_controller rt_controller rt_realtime_utils)
  ament_target_dependencies(test_rt_controller ${THIS_PACKAGE_INCLUDE_DEPENDS})
endif()

# ═══════════════════════════════════════════════════════════════════════════════
# Doxygen (optional)
# ═══════════════════════════════════════════════════════════════════════════════
find_package(Doxygen QUIET)
if(DOXYGEN_FOUND)
  set(DOXYGEN_IN ${CMAKE_CURRENT_SOURCE_DIR}/docs/Doxyfile)
  add_custom_target(doc
    COMMAND ${DOXYGEN_EXECUTABLE} ${DOXYGEN_IN}
    WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR}
    COMMENT "Generating API documentation with Doxygen"
    VERBATIM
  )
endif()

# ─── Export ──────────────────────────────────────────────────────────────────
ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_include_directories(include)
ament_export_libraries(
  rt_realtime_utils
  rt_hardware_interface
  rt_controller
  rt_executor
)
ament_export_dependencies(${THIS_PACKAGE_INCLUDE_DEPENDS})
ament_package()
