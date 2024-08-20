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

#pragma once

#include <palimpsest/Dictionary.h>

#include <limits>
#include <string>
#include <unordered_map>
#include <vector>

#include "upkie/cpp/observers/Observer.h"

/*! Observers used by Upkie.
 *
 * \image html observers.png
 * \image latex observers.eps
 */
namespace upkie::cpp::observers {

using palimpsest::Dictionary;
using upkie::cpp::observers::Observer;

/*! Filter joint variables.
 *
 */
class JointFilter : public Observer {
 public:
  struct Parameters {
    /*! Check whether parameters are incomplete.
     *
     * \returns True if some parameters are uninitialized.
     */
    bool incomplete() { return (dt != dt); }

    //! Spine timestep, in [s]
    double dt = std::numeric_limits<double>::quiet_NaN();

    //! Low-pass filtering time constant, in [s]
    double cutoff_period;

    //! List of upper leg (hip, knee, ...) joints
    std::vector<std::string> upper_leg_joints;

    //! List of wheel joint names
    std::vector<std::string> wheels;

    //! List of measurement variables
    std::vector<std::string> variables;
  };

  /*! Initialize observer.
   *
   * \param[in] params Observer parameters.
   */
  explicit JointFilter(const Parameters& params);

  //! Prefix of outputs in the observation dictionary.
  inline std::string prefix() const noexcept final { return "joint_filter"; }

  /*! Reset observer.
   *
   * \param[in] config Overall configuration dictionary.
   */
  void reset(const Dictionary& config) override;

  /*! Read inputs from other observations.
   *
   * \param[in] observation Dictionary to read other observations from.
   */
  void read(const Dictionary& observation) final;

  /*! Write outputs, called if reading was successful.
   *
   * \param[out] observation Dictionary to write observations to.
   */
  void write(Dictionary& observation) final;

 private:
  //! Observer parameters
  Parameters params_;

  //! Filtered measurement variables
  std::unordered_map<std::string, std::unordered_map<std::string, double>>
      filtered_variables_;

  //! List of joint names
  std::vector<std::string> joints_;
};

}  // namespace upkie::cpp::observers
