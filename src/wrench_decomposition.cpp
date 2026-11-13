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
#include <limits>
#include <stdexcept>

#include "fbml/math_utils.hpp"

namespace fbml
{
namespace math
{

ContactWrenchDecomposer::ContactWrenchDecomposer(Eigen::Index contact_count)
{
  if (contact_count <= 0 || contact_count > std::numeric_limits<Eigen::Index>::max() / 6) {
    throw std::invalid_argument("Contact count must be positive and representable");
  }
  const Eigen::Index size = 6 * contact_count;
  if (
    size >
    std::numeric_limits<Eigen::Index>::max() / static_cast<Eigen::Index>(sizeof(double)) / size) {
    throw std::invalid_argument("Contact workspace size is not representable");
  }
  grasp_.resize(6, 6 * contact_count);
  normalized_.resize(6, contact_count);
  net_scaled_.resize(6 * contact_count);
  result_.net.resize(6, contact_count);
  result_.internal.resize(6, contact_count);
  // NOTE: square zero-padding avoids Eigen's allocating rectangular QR, at O(n^2)
  // storage and O(n^3) worst-case work; use allocation-free thin QR for large contact sets.
  square_grasp_ = Eigen::MatrixXd::Zero(size, size);
  svd_ = decltype(svd_)(size, size, Eigen::ComputeFullU | Eigen::ComputeFullV);
  svd_.setThreshold(6.0 * std::numeric_limits<double>::epsilon());
}

const ContactWrenchDecomposition & ContactWrenchDecomposer::compute(
  const std::vector<Eigen::Isometry3d> & contact_poses,
  const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> & contact_wrenches,
  double characteristic_length)
{
  const Eigen::Index count = contact_wrenches.cols();
  if (
    count != normalized_.cols() || contact_poses.size() != static_cast<std::size_t>(count) ||
    !std::isfinite(characteristic_length) || characteristic_length <= 0.0 ||
    !contact_wrenches.allFinite()) {
    throw std::invalid_argument("Invalid contact wrenches or characteristic length");
  }

  auto & result = result_;
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

  auto & grasp = grasp_;
  auto & normalized = normalized_;
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
  square_grasp_.topRows<6>() = grasp;
  svd_.compute(square_grasp_, Eigen::ComputeFullU | Eigen::ComputeFullV);
  // Full six-axis contact wrenches make G full row rank even for coincident origins.
  if (svd_.rank() != 6) {
    throw std::runtime_error("Scaled contact grasp matrix is numerically rank deficient");
  }
  Eigen::Vector<double, 6> coefficients =
    svd_.matrixU().topLeftCorner<6, 6>().transpose() * resultant_scaled;
  coefficients.array() /= svd_.singularValues().head<6>().array();
  net_scaled_.noalias() = svd_.matrixV().leftCols<6>() * coefficients;
  const auto & net_scaled = net_scaled_;
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

ContactWrenchDecomposition decomposeContactWrenches(
  const std::vector<Eigen::Isometry3d> & contact_poses,
  const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> & contact_wrenches,
  double characteristic_length)
{
  ContactWrenchDecomposer decomposer(contact_wrenches.cols());
  return decomposer.compute(contact_poses, contact_wrenches, characteristic_length);
}

}  // namespace math
}  // namespace fbml
