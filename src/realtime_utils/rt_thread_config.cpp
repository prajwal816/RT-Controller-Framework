/**
 * @file rt_thread_config.cpp
 * @brief Implementation of real-time thread configuration utilities.
 *
 * Linux: Uses sched_setscheduler, mlockall, sched_setaffinity.
 * Other platforms: No-op stubs that return informative messages.
 *
 * @copyright Copyright (c) 2024. Apache-2.0 License.
 */

#include "rt_controller_framework/realtime_utils/rt_thread_config.hpp"

#include <sstream>

#ifdef __linux__
#include <pthread.h>
#include <sched.h>
#include <sys/mman.h>
#include <sys/utsname.h>
#include <unistd.h>
#include <cerrno>
#include <cstring>
#include <fstream>
#endif

namespace rt_controller_framework {
namespace realtime_utils {

ThreadConfigResult set_thread_rt_priority(int priority) {
  ThreadConfigResult result;

#ifdef __linux__
  if (priority < 1 || priority > 99) {
    result.success = false;
    result.message = "RT priority must be between 1 and 99, got " +
                     std::to_string(priority);
    return result;
  }

  struct sched_param param;
  param.sched_priority = priority;

  int ret = sched_setscheduler(0, SCHED_FIFO, &param);
  if (ret == 0) {
    result.success = true;
    result.message = "Set SCHED_FIFO priority " + std::to_string(priority);
  } else {
    result.success = false;
    result.message = "Failed to set RT priority: " +
                     std::string(std::strerror(errno)) +
                     " (try running with sudo or CAP_SYS_NICE)";
  }
#else
  result.success = false;
  result.message = "RT thread priority not available on this platform "
                   "(Linux required)";
  (void)priority;
#endif

  return result;
}

ThreadConfigResult lock_memory() {
  ThreadConfigResult result;

#ifdef __linux__
  int ret = mlockall(MCL_CURRENT | MCL_FUTURE);
  if (ret == 0) {
    result.success = true;
    result.message = "Memory locked (MCL_CURRENT | MCL_FUTURE)";
  } else {
    result.success = false;
    result.message = "Failed to lock memory: " +
                     std::string(std::strerror(errno)) +
                     " (try running with sudo or CAP_IPC_LOCK)";
  }
#else
  result.success = false;
  result.message = "Memory locking not available on this platform "
                   "(Linux required)";
#endif

  return result;
}

ThreadConfigResult set_cpu_affinity(const std::vector<int>& cpu_ids) {
  ThreadConfigResult result;

  if (cpu_ids.empty()) {
    result.success = true;
    result.message = "No CPU affinity change requested";
    return result;
  }

#ifdef __linux__
  cpu_set_t cpuset;
  CPU_ZERO(&cpuset);

  int max_cpus = sysconf(_SC_NPROCESSORS_ONLN);
  std::ostringstream cores_str;

  for (int cpu : cpu_ids) {
    if (cpu < 0 || cpu >= max_cpus) {
      result.success = false;
      result.message = "Invalid CPU ID " + std::to_string(cpu) +
                       " (system has " + std::to_string(max_cpus) + " cores)";
      return result;
    }
    CPU_SET(cpu, &cpuset);
    if (cores_str.str().length() > 0) cores_str << ",";
    cores_str << cpu;
  }

  int ret = sched_setaffinity(0, sizeof(cpu_set_t), &cpuset);
  if (ret == 0) {
    result.success = true;
    result.message = "Pinned to CPU core(s): " + cores_str.str();
  } else {
    result.success = false;
    result.message = "Failed to set CPU affinity: " +
                     std::string(std::strerror(errno));
  }
#else
  result.success = false;
  result.message = "CPU affinity not available on this platform "
                   "(Linux required)";
  (void)cpu_ids;
#endif

  return result;
}

bool configure_rt_thread(int priority, const std::vector<int>& cpu_ids) {
  bool all_ok = true;

  // Step 1: Lock memory
  auto mem_result = lock_memory();
  if (!mem_result.success) {
    all_ok = false;
  }

  // Step 2: Set RT priority
  auto prio_result = set_thread_rt_priority(priority);
  if (!prio_result.success) {
    all_ok = false;
  }

  // Step 3: Set CPU affinity
  if (!cpu_ids.empty()) {
    auto aff_result = set_cpu_affinity(cpu_ids);
    if (!aff_result.success) {
      all_ok = false;
    }
  }

  return all_ok;
}

std::string get_rt_system_info() {
  std::ostringstream info;
  info << "=== RT System Information ===\n";

#ifdef __linux__
  // Kernel version
  struct utsname uname_buf;
  if (uname(&uname_buf) == 0) {
    info << "Kernel:   " << uname_buf.release << "\n";
    info << "System:   " << uname_buf.sysname << " " << uname_buf.machine
         << "\n";

    // Check for PREEMPT_RT
    std::string release(uname_buf.release);
    bool has_rt = (release.find("rt") != std::string::npos) ||
                  (release.find("RT") != std::string::npos) ||
                  (release.find("PREEMPT_RT") != std::string::npos);
    info << "RT Kernel: " << (has_rt ? "YES (PREEMPT_RT detected)" : "NO")
         << "\n";
  }

  // CPU count
  int ncpus = sysconf(_SC_NPROCESSORS_ONLN);
  info << "CPUs:     " << ncpus << " online\n";

  // Check for isolated CPUs
  std::ifstream isolcpus("/sys/devices/system/cpu/isolated");
  if (isolcpus.is_open()) {
    std::string isolated;
    std::getline(isolcpus, isolated);
    info << "Isolated: " << (isolated.empty() ? "none" : isolated) << "\n";
  }

  // Max RT priority
  int max_prio = sched_get_priority_max(SCHED_FIFO);
  info << "Max FIFO: " << max_prio << "\n";

  // Current scheduler
  int policy = sched_getscheduler(0);
  const char* policy_name = "UNKNOWN";
  switch (policy) {
    case SCHED_OTHER: policy_name = "SCHED_OTHER (non-RT)"; break;
    case SCHED_FIFO:  policy_name = "SCHED_FIFO (RT)"; break;
    case SCHED_RR:    policy_name = "SCHED_RR (RT)"; break;
  }
  info << "Scheduler: " << policy_name << "\n";

#else
  info << "Platform: Non-Linux (RT features unavailable)\n";
#endif

  return info.str();
}

}  // namespace realtime_utils
}  // namespace rt_controller_framework
