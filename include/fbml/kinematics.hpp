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
#include <tuple>
#include <vector>

#include <pinocchio/multibody/data.hpp>

#include "fbml/core.hpp"
#include "fbml/visibility_control.h"

namespace fbml
{

struct IKSettings
{
  double tolerance = 1e-4;
  int max_iterations = 100;
  double damping_factor = 1e-2;

  // Weights for errors on each axis [vx, vy, vz, wx, wy, wz]
  Eigen::Vector<double, 6> task_weights = Eigen::Vector<double, 6>::Ones();
};

class FBML_PUBLIC Kinematics
{
public:
  explicit Kinematics(const RobotCore & core);
  virtual ~Kinematics() = default;

  Eigen::MatrixXd computeJacobian(
    const Eigen::VectorXd & q, const std::string & frame_name,
    pinocchio::ReferenceFrame reference_frame = pinocchio::LOCAL);

  /**
   * @brief Compute the manipulability measure for a given joint configuration and task dimensions.
   *
   * @return The manipulability measure.
   */
  double computeManipulability(
    const Eigen::VectorXd & q, const std::string & frame_name,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims);

  /**
   * @brief Compute the manipulability measure + ellipsoid for a given joint configuration and task dimensions.
   *
   * @return The manipulability measure.
   */
  double computeManipulability(
    const Eigen::VectorXd & q, const std::string & frame_name,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
    Eigen::VectorXd & eigenvalues_out, Eigen::MatrixXd & eigenvectors_out);

  double computeBaseManipulability(
    const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims);

  double computeBaseManipulability(
    const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
    Eigen::VectorXd & eigenvalues_out, Eigen::MatrixXd & eigenvectors_out);

  Eigen::Isometry3d solveFK(
    const Eigen::VectorXd & q, const std::string & target_frame,
    const std::string & reference_frame = "world");

  bool solveNumericalIK(
    Eigen::VectorXd & q, const std::string & frame_name, const Eigen::Isometry3d & desired_pose,
    const std::vector<std::string> & joint_names, const std::string & reference_frame = "world",
    const IKSettings & settings = IKSettings());

  bool solvePositionIK(
    Eigen::VectorXd & q, const std::string & frame_name, const Eigen::Vector3d & desired_position,
    const std::vector<std::string> & joint_names, const std::string & reference_frame = "world",
    IKSettings settings = IKSettings())
  {
    settings.task_weights << 1.0, 1.0, 1.0, 0.0, 0.0, 0.0;  // Only for position

    Eigen::Isometry3d desired_pose = Eigen::Isometry3d::Identity();
    desired_pose.translation() = desired_position;

    return solveNumericalIK(q, frame_name, desired_pose, joint_names, reference_frame, settings);
  }

  void solveIVK(
    const Eigen::VectorXd & q, const std::string & frame_name,
    const Eigen::Vector<double, 6> & desired_twist_in_local,
    const std::vector<std::string> & joint_names, Eigen::Ref<Eigen::VectorXd> joint_vel_out,
    double damping_factor = 1e-2);

private:
  /**
   * @brief Compute the frame Jacobian expressed in the given reference frame.
   */
  void computeFrameJacobianInto(
    const Eigen::VectorXd & q, const std::string & frame_name,
    pinocchio::ReferenceFrame reference_frame);

  int assembleSubJacobian(const std::vector<std::string> & joint_names);

  double computeManipulabilityCore(
    const Eigen::VectorXd & q, const std::string & frame_name,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims);

  double computeBaseManipulabilityCore(
    const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
    const std::vector<std::string> & joint_names, const std::vector<int> & task_dims);

  const pinocchio::Model & model_;
  pinocchio::Data data_;
  const RobotCore & core_;

  // --- Preallocated workspaces ---

  pinocchio::Data::Matrix6x j_ac_;     // 6 x nv frame Jacobian
  Eigen::MatrixXd j_sub_;              // 6 x nv sub-Jacobian
  Eigen::Matrix<double, 6, 6> dls_A_;  // solveIVK DLS system matrix

  Eigen::MatrixXd manip_j_task_;           // 6 x nv
  Eigen::Matrix<double, 6, 6> manip_jjt_;  // J J^T
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> manip_eig_;

  Eigen::MatrixXd bmanip_jb_;              // 6k x 6
  Eigen::MatrixXd bmanip_jq_;              // 6k x sub_nv
  Eigen::Matrix<double, 6, 6> bmanip_vd_;  // V * Sigma^-1
  Eigen::MatrixXd bmanip_pinv_;            // 6 x 6k
  Eigen::MatrixXd bmanip_jeq_;             // 6 x sub_nv
  Eigen::MatrixXd bmanip_jeq_task_;        // 6 x nv
  Eigen::JacobiSVD<Eigen::MatrixXd> bmanip_svd_;
};

}  // namespace fbml

#endif  // FBML__KINEMATICS_HPP_
