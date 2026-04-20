// Copyright (c) 2026 Masazumi Imai
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

#ifndef FBML__CORE_HPP_
#define FBML__CORE_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <pinocchio/multibody/model.hpp>

#include "fbml/visibility_control.h"

namespace fbml
{

class FBML_PUBLIC RobotCore
{
public:
  explicit RobotCore(
    const std::string & urdf_path,
    const Eigen::Vector3d & gravity = Eigen::Vector3d(0.0, 0.0, -9.81));
  virtual ~RobotCore() = default;

  const pinocchio::Model & getModel() const { return model_; }

  void setActuatorParameters(double armature, double damping);

  void setActuatorParameters(
    const Eigen::VectorXd & armature_vector, const Eigen::VectorXd & damping_vector);

  std::vector<std::string> getJointNamesBetweenFrames(
    const std::string & start_frame_name, const std::string & end_frame_name) const;

private:
  pinocchio::Model model_;
};

}  // namespace fbml

#endif  // FBML__CORE_HPP_
