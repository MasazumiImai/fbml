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

#include <pinocchio/algorithm/centroidal.hpp>
#include <pinocchio/algorithm/crba.hpp>
#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/rnea.hpp>

namespace fbml
{

Dynamics::Dynamics(const RobotCore & core)
: model_(core.getModel()),
  data_(pinocchio::Data(model_)),
  core_(core),
  frame_jacobian_(pinocchio::Data::Matrix6x::Zero(6, model_.nv)),
  base_coupling_(Eigen::MatrixXd::Zero(6, model_.nv - 6))
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
  const Eigen::VectorXd & q, Eigen::Matrix<double, 6, 6> & M_b, Eigen::MatrixXd & M_bm)
{
  pinocchio::crba(model_, data_, q);
  data_.M.triangularView<Eigen::StrictlyLower>() =
    data_.M.transpose().triangularView<Eigen::StrictlyLower>();

  const int njoints = model_.nv - 6;

  M_b = data_.M.block<6, 6>(0, 0);
  M_bm = data_.M.block(0, 6, 6, njoints);
}

Eigen::VectorXd Dynamics::computeNonLinearEffects(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v)
{
  // Non-linear term (coriolis + gravity) (RNEA: Recursive Newton-Euler Algorithm)
  Eigen::VectorXd nle = pinocchio::nonLinearEffects(model_, data_, q, v);

  // Viscous joint damping (tau = damping .* v); RNEA does not include it.
  nle += model_.damping.cwiseProduct(v);

  return nle;
}

void Dynamics::computeCentroidalMomentum(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v, Eigen::Vector<double, 6> & momentum,
  Eigen::Vector3d & com)
{
  momentum = pinocchio::computeCentroidalMomentum(model_, data_, q, v).toVector();
  com = data_.com[0];
}

Eigen::MatrixXd Dynamics::computeGeneralizedJacobian(
  const Eigen::VectorXd & q, const std::string & frame_name,
  pinocchio::ReferenceFrame reference_frame)
{
  Eigen::MatrixXd jacobian(6, model_.nv - 6);
  computeGeneralizedJacobians(q, {frame_name}, jacobian, reference_frame);
  return jacobian;
}

void Dynamics::computeGeneralizedJacobians(
  const Eigen::VectorXd & q, const std::vector<std::string> & frame_names,
  Eigen::Ref<Eigen::MatrixXd> jacobians_out, pinocchio::ReferenceFrame reference_frame)
{
  const int njoints = model_.nv - 6;
  if (
    jacobians_out.rows() != static_cast<Eigen::Index>(6 * frame_names.size()) ||
    jacobians_out.cols() != njoints) {
    throw std::invalid_argument("jacobians_out must be (6 * frames) x (nv - 6).");
  }

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);
  pinocchio::crba(model_, data_, q);

  // J* = J_m - J_b * M_b^{-1} * M_bm; the base coupling term is shared by every frame.
  base_inertia_llt_.compute(data_.M.topLeftCorner<6, 6>());
  base_coupling_ = data_.M.topRightCorner(6, njoints);
  base_inertia_llt_.solveInPlace(base_coupling_);

  for (std::size_t index = 0; index < frame_names.size(); ++index) {
    frame_jacobian_.setZero();
    pinocchio::getFrameJacobian(
      model_, data_, core_.frameId(frame_names[index]), reference_frame, frame_jacobian_);
    auto jacobian = jacobians_out.middleRows(static_cast<Eigen::Index>(6 * index), 6);
    jacobian = frame_jacobian_.rightCols(njoints);
    jacobian.noalias() -= frame_jacobian_.leftCols<6>() * base_coupling_;
  }
}

}  // namespace fbml
