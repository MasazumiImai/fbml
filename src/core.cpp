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

std::vector<std::string> RobotCore::getJointNamesBetweenFrames(
  const std::string & start_frame_name, const std::string & end_frame_name) const
{
  if (!model_.existFrame(start_frame_name)) {
    throw std::invalid_argument("Frame '" + start_frame_name + "' does not exist.");
  }
  if (!model_.existFrame(end_frame_name)) {
    throw std::invalid_argument("Frame '" + end_frame_name + "' does not exist.");
  }

  pinocchio::FrameIndex start_frame_id = model_.getFrameId(start_frame_name);
  pinocchio::FrameIndex end_frame_id = model_.getFrameId(end_frame_name);

  pinocchio::JointIndex start_joint = model_.frames[start_frame_id].parentJoint;
  pinocchio::JointIndex end_joint = model_.frames[end_frame_id].parentJoint;

  auto getPathToRoot = [this](pinocchio::JointIndex j) {
    std::vector<pinocchio::JointIndex> path;
    while (j > 0) {
      path.push_back(j);
      j = model_.parents[j];
    }
    return path;
  };

  std::vector<pinocchio::JointIndex> path_start = getPathToRoot(start_joint);
  std::vector<pinocchio::JointIndex> path_end = getPathToRoot(end_joint);

  int idx_start = path_start.size() - 1;
  int idx_end = path_end.size() - 1;

  // Find common ancestor
  while (idx_start >= 0 && idx_end >= 0 && path_start[idx_start] == path_end[idx_end]) {
    idx_start--;
    idx_end--;
  }

  std::vector<pinocchio::JointIndex> final_path_ids;

  // Start to common ancestor
  for (int i = 0; i <= idx_start; ++i) {
    final_path_ids.push_back(path_start[i]);
  }
  // Common ancestor to end
  for (int i = idx_end; i >= 0; --i) {
    final_path_ids.push_back(path_end[i]);
  }

  std::vector<std::string> joint_names;
  for (const auto & j_id : final_path_ids) {
    joint_names.push_back(model_.names[j_id]);
  }

  return joint_names;
}

}  // namespace fbml
