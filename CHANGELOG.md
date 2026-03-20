# Changelog

All notable changes to the RT Controller Framework will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2024-01-01

### Added

#### Core Framework
- **Lock-Free SPSC Queue** — atomic acquire/release ring buffer with `alignas(64)` cache-line separation
- **Pre-allocated Memory Pool** — O(1) alloc/free via intrusive free-list, bounds-checked deallocation
- **High-Resolution Timer** — `steady_clock` + `sleep_until` with overrun detection and cascading delay recovery
- **Jitter Monitor** — ring-buffer based statistics (mean/max/stddev), overrun counting, formatted reporting

#### ros2_control Integration
- **MockActuatorSystem** — `hardware_interface::SystemInterface` plugin with position/velocity/effort interfaces and Euler-integrated dynamics simulation
- **RTController** — `controller_interface::ControllerInterface` plugin with zero-allocation `update()`, lock-free command queue, PD control, jitter self-monitoring
- Plugin XML descriptors for `pluginlib` registration

#### RT Thread Configuration
- **RT Thread Config Utility** — `SCHED_FIFO` priority, `mlockall()`, CPU affinity pinning with portable no-op fallback for non-Linux platforms
- System diagnostics: kernel version, PREEMPT_RT detection, isolated CPUs, scheduler policy

#### Infrastructure
- **RT Executor Node** — standalone demo with dedicated RT thread, 1kHz loop, `/rt_metrics` publisher
- Example `rt_loop_example` demonstrating all RT utilities
- YAML configurations for controller gains and hardware parameters
- Python launch files for ros2_control integration and standalone demo
- Standalone URDF (`urdf/mock_robot.urdf`) with proper visual/collision/inertial elements

#### Testing & CI
- 6 GTest suites covering all components
- GitHub Actions CI workflow (build + test + lint on ROS2 Humble Docker)

#### Documentation
- Comprehensive README with ASCII architecture diagram
- Doxygen API documentation configuration
- Apache-2.0 license
