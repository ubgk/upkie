/*
 * Copyright 2023 Inria
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include "upkie/observers/JointFilter.h"

#include <palimpsest/exceptions/KeyError.h>

#include "vulp/utils/low_pass_filter.h"

namespace upkie::observers {

using palimpsest::exceptions::KeyError;
using vulp::utils::low_pass_filter;

JointFilter::JointFilter(const Parameters& params)
    : params_(params) {
  
  // Reserve space for the list of joints
  joints_.reserve(params_.upper_leg_joints.size() + params_.wheels.size());

  // Append upper leg joints to the list of joints
  joints_.insert(joints_.end(), params_.upper_leg_joints.begin(), params_.upper_leg_joints.end());

  // Append wheel joints to the list of joints
  joints_.insert(joints_.end(), params_.wheels.begin(), params_.wheels.end());
}


void JointFilter::reset(const Dictionary& config) {
  // Reset filtered variables
  filtered_variables_.clear();
}

void JointFilter::read(const Dictionary& observation) {
  // Get the current time
  const auto start = std::chrono::steady_clock::now();

  if (params_.incomplete()) {
    spdlog::warn("[JointFilter] Observer has not been configured yet!");
    return;
  }
  if (!observation.has("servo")) {
    return;
  }
  
  for (const auto& joint : joints_) {
    for (const auto& variable : params_.variables) {
      if (!observation("servo")(joint).has(variable)) {
        spdlog::warn("[JointFilter] Missing variable {} for joint {}", variable, joint);
        return;
      }
      // read variable
      const double reading = observation("servo")(joint)(variable);

      // filter variable
      filtered_variables_[joint][variable] 
        = low_pass_filter(filtered_variables_[joint][variable], params_.cutoff_period, 
        std::abs(reading), params_.dt);

    }
  }
  // Get the current time
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  spdlog::debug("[JointFilter] read() took {} us", elapsed.count());
}

void JointFilter::write(Dictionary& observation) {
  // Get the current time
  const auto start = std::chrono::steady_clock::now();

  auto& output = observation(prefix());
  
  for (const auto& joint : joints_) {
    output(joint) = Dictionary();
    auto& joint_output = output(joint);

    for (const auto& variable : params_.variables) {
      joint_output(variable) = filtered_variables_[joint][variable];
    }
  }

  // Get the current time
  const auto end = std::chrono::steady_clock::now();
  const auto elapsed = std::chrono::duration_cast<std::chrono::microseconds>(end - start);
  spdlog::debug("[JointFilter] write() took {} us", elapsed.count());
}

}  // namespace upkie::observers
