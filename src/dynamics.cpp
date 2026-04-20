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

#include "fbml/dynamics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace fbml
{

Dynamics::Dynamics(const RobotCore & core) : model_(core.getModel()), data_(pinocchio::Data(model_))
{
}

Eigen::MatrixXd Dynamics::computeMassMatrix(const Eigen::VectorXd & q)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();
  return data_.M;
}

void Dynamics::computePartitionedMassMatrices(
  const Eigen::VectorXd & q, Eigen::MatrixXd & M_b, Eigen::MatrixXd & M_bm)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  const int njoints = model_.nv - 6;

  M_b = data_.M.block(0, 0, 6, 6);
  M_bm = data_.M.block(0, 6, 6, njoints);
}

Eigen::VectorXd Dynamics::computeNonLinearEffects(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  // Non-linear term (coriolis + gravity) (RNEA: Recursive Newton-Euler Algorithm)
  return pinocchio::nonLinearEffects(model_, data_, q, v);
}

Eigen::MatrixXd Dynamics::computeGeneralizedJacobian(
  const Eigen::VectorXd & q, const std::string & frame_name,
  pinocchio::ReferenceFrame reference_frame)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist.");
  }
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  // Inertial matrix M (Composite Rigid Body Algorithm)
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  pinocchio::Data::Matrix6x J_full(6, model_.nv);
  J_full.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, reference_frame, J_full);

  const int njoints = model_.nv - 6;

  fbml::Matrix6d M_b = data_.M.block<6, 6>(0, 0);
  Eigen::MatrixXd M_bm = data_.M.block(0, 6, njoints, 6).transpose();

  fbml::Matrix6d J_b = J_full.block<6, 6>(0, 0);
  Eigen::MatrixXd J_m = J_full.block(0, 6, 6, njoints);

  // J* = J_m - J_b * M_b^{-1} * M_bm
  Eigen::MatrixXd J_g = J_m - J_b * M_b.llt().solve(M_bm);

  return J_g;
}

}  // namespace fbml
