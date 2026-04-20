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

#ifndef FBML__KINEMATICS_HPP_
#define FBML__KINEMATICS_HPP_

#include <Eigen/Dense>
#include <string>
#include <vector>

#include <pinocchio/multibody/data.hpp>
#include <pinocchio/spatial/se3.hpp>

#include "fbml/core.hpp"
#include "fbml/types.hpp"
#include "fbml/visibility_control.h"

namespace fbml
{

struct IKSettings
{
  double tolerance = 1e-4;
  int max_iterations = 100;
  double damping_factor = 1e-2;

  // Weights for errors on each axis [vx, vy, vz, wx, wy, wz]
  fbml::Vector6d task_weights = fbml::Vector6d::Ones();
};

class FBML_PUBLIC Kinematics
{
public:
  explicit Kinematics(const RobotCore & core);
  virtual ~Kinematics() = default;

  Eigen::MatrixXd computeJacobian(
    const Eigen::VectorXd & q, const std::string & frame_name,
    pinocchio::ReferenceFrame reference_frame = pinocchio::LOCAL);

  pinocchio::SE3 solveFK(
    const Eigen::VectorXd & q, const std::string & target_frame,
    const std::string & reference_frame = "world");

  bool solveNumericalIK(
    Eigen::VectorXd & q, const std::string & frame_name, const pinocchio::SE3 & desired_pose,
    const std::vector<std::string> & joint_names, const std::string & reference_frame = "world",
    const IKSettings & settings = IKSettings());

  bool solveNumericalIK(
    Eigen::VectorXd & q, const std::string & frame_name, const Eigen::Isometry3d & desired_pose,
    const std::vector<std::string> & joint_names, const std::string & reference_frame = "world",
    const IKSettings & settings = IKSettings())
  {
    pinocchio::SE3 pose_se3(desired_pose.matrix());
    return solveNumericalIK(q, frame_name, pose_se3, joint_names, reference_frame, settings);
  }

  bool solvePositionIK(
    Eigen::VectorXd & q, const std::string & frame_name, const Eigen::Vector3d & desired_position,
    const std::vector<std::string> & joint_names, const std::string & reference_frame = "world",
    IKSettings settings = IKSettings())
  {
    settings.task_weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;  // Only for position

    pinocchio::SE3 desired_pose(Eigen::Matrix3d::Identity(), desired_position);

    return solveNumericalIK(q, frame_name, desired_pose, joint_names, reference_frame, settings);
  }

private:
  const pinocchio::Model & model_;
  pinocchio::Data data_;
};

}  // namespace fbml

#endif  // FBML__KINEMATICS_HPP_
