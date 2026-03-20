// Copyright 2024 RT Controller Framework Contributors
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

/// @file jitter_monitor.cpp
/// @brief Explicit template instantiations for JitterMonitor.

#include "rt_controller_framework/realtime_utils/jitter_monitor.hpp"

namespace rt_controller_framework
{
namespace realtime_utils
{

// Explicit instantiation for the default buffer size
template class JitterMonitor<1000>;

}  // namespace realtime_utils
}  // namespace rt_controller_framework
