/**
 * @file jitter_monitor.cpp
 * @brief Jitter monitor — explicit template instantiation.
 *
 * The JitterMonitor is a header-only template class. This translation
 * unit exists to provide the library symbol for the default instantiation
 * and to ensure the shared library links correctly.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"

namespace rt_controller_framework {
namespace realtime_utils {

// Explicit instantiation of the default buffer size (1000 samples)
template class JitterMonitor<1000>;

// Also instantiate a smaller buffer for testing
template class JitterMonitor<10>;
template class JitterMonitor<100>;

}  // namespace realtime_utils
}  // namespace rt_controller_framework
