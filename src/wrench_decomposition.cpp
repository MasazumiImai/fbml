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

#include <cmath>
#include <stdexcept>

#include "fbml/math_utils.hpp"

namespace fbml
{
namespace math
{

ContactWrenchDecomposition decomposeContactWrenches(
  const std::vector<Eigen::Isometry3d> & contact_poses,
  const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> & contact_wrenches,
  double characteristic_length)
{
  const Eigen::Index count = contact_wrenches.cols();
  if (
    count == 0 || contact_poses.size() != static_cast<std::size_t>(count) ||
    !std::isfinite(characteristic_length) || characteristic_length <= 0.0 ||
    !contact_wrenches.allFinite()) {
    throw std::invalid_argument("Invalid contact wrenches or characteristic length");
  }

  ContactWrenchDecomposition result;
  result.reference_point.setZero();
  for (const auto & pose : contact_poses) {
    if (
      !pose.matrix().allFinite() ||
      !pose.matrix().row(3).isApprox(Eigen::RowVector4d(0.0, 0.0, 0.0, 1.0), 1e-8) ||
      !(pose.linear().transpose() * pose.linear()).isApprox(Eigen::Matrix3d::Identity(), 1e-8) ||
      std::abs(pose.linear().determinant() - 1.0) > 1e-8) {
      throw std::invalid_argument("Contact pose must be a finite rigid transform");
    }
    result.reference_point += pose.translation() / static_cast<double>(count);
  }

  // NOTE: per-call allocation keeps the calculation independent of contact count. Add a
  // preallocated workspace before integrating this function into a hard real-time controller.
  Eigen::Matrix<double, 6, Eigen::Dynamic> grasp(6, 6 * count);
  Eigen::Matrix<double, 6, Eigen::Dynamic> normalized(6, count);
  for (Eigen::Index i = 0; i < count; ++i) {
    const auto & pose = contact_poses[i];
    const Eigen::Vector3d r = (pose.translation() - result.reference_point) / characteristic_length;
    Eigen::Matrix3d cross;
    cross << 0.0, -r.z(), r.y(), r.z(), 0.0, -r.x(), -r.y(), r.x(), 0.0;
    grasp.block<3, 3>(0, 6 * i).setIdentity();
    grasp.block<3, 3>(0, 6 * i + 3).setZero();
    grasp.block<3, 3>(3, 6 * i) = cross;
    grasp.block<3, 3>(3, 6 * i + 3).setIdentity();
    normalized.col(i).head<3>() = pose.linear() * contact_wrenches.col(i).head<3>();
    normalized.col(i).tail<3>() =
      pose.linear() * (contact_wrenches.col(i).tail<3>() / characteristic_length);
  }
  if (!grasp.allFinite() || !normalized.allFinite()) {
    throw std::runtime_error("Contact wrench normalization overflowed");
  }

  const Eigen::Map<const Eigen::VectorXd> normalized_vector(normalized.data(), normalized.size());
  const Eigen::Vector<double, 6> resultant_scaled = grasp * normalized_vector;
  if (!resultant_scaled.allFinite()) {
    throw std::runtime_error("Contact wrench resultant overflowed");
  }
  const Eigen::JacobiSVD<Eigen::Matrix<double, 6, Eigen::Dynamic>> svd(
    grasp, Eigen::ComputeThinU | Eigen::ComputeThinV);
  // Full six-axis contact wrenches make G full row rank even for coincident origins.
  if (svd.rank() != 6) {
    throw std::runtime_error("Scaled contact grasp matrix is numerically rank deficient");
  }
  const Eigen::VectorXd net_scaled = svd.solve(resultant_scaled);
  if (
    !net_scaled.allFinite() ||
    (grasp * net_scaled - resultant_scaled).stableNorm() >
      1e-10 * (1.0 + resultant_scaled.stableNorm())) {
    throw std::runtime_error("Contact wrench decomposition did not preserve the resultant");
  }

  result.resultant = resultant_scaled;
  result.resultant.tail<3>() *= characteristic_length;
  result.net.resize(6, count);
  for (Eigen::Index i = 0; i < count; ++i) {
    const auto rotation_to_contact = contact_poses[i].linear().transpose();
    result.net.col(i).head<3>() = rotation_to_contact * net_scaled.segment<3>(6 * i);
    result.net.col(i).tail<3>() =
      rotation_to_contact * (characteristic_length * net_scaled.segment<3>(6 * i + 3));
  }
  result.internal = contact_wrenches - result.net;
  if (!result.resultant.allFinite() || !result.net.allFinite() || !result.internal.allFinite()) {
    throw std::runtime_error("Contact wrench denormalization overflowed");
  }
  return result;
}

}  // namespace math
}  // namespace fbml
