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

#include "fbml/core.hpp"

#include <iostream>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>

namespace fbml
{

RobotCore::RobotCore(const std::string & urdf_path, const Eigen::Vector3d & gravity)
{
  try {
    pinocchio::urdf::buildModel(urdf_path, pinocchio::JointModelFreeFlyer(), model_);

    model_.gravity.linear() = gravity;

    std::cout << "[fbml] Successfully loaded URDF." << std::endl;
    std::cout << "[fbml] - Number of joints: " << model_.njoints << std::endl;
    std::cout << "[fbml] - Degrees of freedom (nv): " << model_.nv << std::endl;
    std::cout << "[fbml] - Configuration size (nq): " << model_.nq << std::endl;

  } catch (const std::exception & e) {
    std::cerr << "[fbml] Failed to build Pinocchio model from URDF: " << urdf_path << std::endl;
    std::cerr << e.what() << std::endl;
    throw;
  }
}

void RobotCore::setActuatorParameters(double armature, double damping)
{
  const int num_actuated_joints = model_.nv - 6;

  if (num_actuated_joints > 0) {
    model_.rotorInertia.tail(num_actuated_joints).setConstant(armature);
    model_.friction.tail(num_actuated_joints).setConstant(damping);
  }
}

void RobotCore::setActuatorParameters(
  const Eigen::VectorXd & armature_vector, const Eigen::VectorXd & damping_vector)
{
  const int num_actuated_joints = model_.nv - 6;

  if (
    armature_vector.size() != num_actuated_joints || damping_vector.size() != num_actuated_joints) {
    throw std::invalid_argument(
      "[fbml] Size of parameter vectors must match the number of actuated joints (" +
      std::to_string(num_actuated_joints) + ").");
  }

  model_.rotorInertia.tail(num_actuated_joints) = armature_vector;
  model_.friction.tail(num_actuated_joints) = damping_vector;
}

}  // namespace fbml
