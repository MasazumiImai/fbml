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

#include <iostream>
#include <limits>
#include <stdexcept>

#include "fbml/math_utils.hpp"

using Wrenches = Eigen::Matrix<double, 6, Eigen::Dynamic>;
using Poses = std::vector<Eigen::Isometry3d>;
using fbml::math::decomposeContactWrenches;

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

// Independent physical summation: rotate at each contact, then shift to the common point.
Eigen::Vector<double, 6> resultant(
  const Poses & poses, const Wrenches & wrenches, const Eigen::Vector3d & point)
{
  Eigen::Vector<double, 6> sum = Eigen::Vector<double, 6>::Zero();
  for (Eigen::Index i = 0; i < wrenches.cols(); ++i) {
    const Eigen::Vector3d force = poses[i].linear() * wrenches.col(i).head<3>();
    sum.head<3>() += force;
    sum.tail<3>() +=
      poses[i].linear() * wrenches.col(i).tail<3>() + (poses[i].translation() - point).cross(force);
  }
  return sum;
}

void checkDecomposition(const Poses & poses, const Wrenches & wrenches, double length)
{
  const auto split = decomposeContactWrenches(poses, wrenches, length);
  require((split.net + split.internal).isApprox(wrenches, 1e-12), "Reconstruction failed");
  const auto expected = resultant(poses, wrenches, split.reference_point);
  require(split.resultant.isApprox(expected, 1e-12), "Reported resultant changed");
  require(
    (resultant(poses, split.net, split.reference_point) - expected).norm() < 1e-10,
    "Minimum-norm component changed the resultant");
  require(
    resultant(poses, split.internal, split.reference_point).norm() < 1e-10,
    "Internal component has nonzero resultant");
  Wrenches net = split.net;
  Wrenches internal = split.internal;
  net.bottomRows<3>() /= length;
  internal.bottomRows<3>() /= length;
  require(
    std::abs(net.cwiseProduct(internal).sum()) < 1e-10,
    "Minimum-norm and internal components are not orthogonal");
  require(
    (net + internal).squaredNorm() >= net.squaredNorm() - 1e-10,
    "Decomposition increased the normalized norm");

  // A change of reference axes/origin must not change local contact components.
  Eigen::Isometry3d transform = Eigen::Isometry3d::Identity();
  transform.linear() =
    Eigen::AngleAxisd(0.8, Eigen::Vector3d(1, 2, -3).normalized()).toRotationMatrix();
  transform.translation() = Eigen::Vector3d(2, -3, 4);
  Poses moved = poses;
  for (auto & pose : moved) pose = transform * pose;
  const auto rotated = decomposeContactWrenches(moved, wrenches, length);
  require((rotated.net - split.net).norm() < 1e-10, "Reference frame changed net component");
  require(
    (rotated.internal - split.internal).norm() < 1e-10,
    "Reference frame changed internal component");
  require(
    rotated.reference_point.isApprox(transform * split.reference_point, 1e-12),
    "Reference point did not follow the rigid transform");
  require(
    (rotated.resultant.head<3>() - transform.linear() * split.resultant.head<3>()).norm() < 1e-10 &&
      (rotated.resultant.tail<3>() - transform.linear() * split.resultant.tail<3>()).norm() < 1e-10,
    "Resultant did not rotate with the reference frame");

  // Changing length units (m -> mm) must preserve the physical decomposition.
  Poses millimeters = poses;
  for (auto & pose : millimeters) pose.translation() *= 1000;
  Wrenches scaled = wrenches;
  scaled.bottomRows<3>() *= 1000;
  auto converted = decomposeContactWrenches(millimeters, scaled, length * 1000);
  converted.net.bottomRows<3>() /= 1000;
  converted.internal.bottomRows<3>() /= 1000;
  require((converted.net - split.net).norm() < 1e-10, "Length units changed net component");
  require(
    (converted.internal - split.internal).norm() < 1e-10,
    "Length units changed internal component");
}

int main()
{
  constexpr double length = 0.41662;
  for (const int count : {1, 3, 4}) {
    Poses poses(count, Eigen::Isometry3d::Identity());
    Wrenches wrenches(6, count);
    for (int i = 0; i < count; ++i) {
      poses[i].translation() = Eigen::Vector3d(0.2 * i, 0.3 * (i % 2), -0.1 * i * i);
      poses[i].linear() =
        Eigen::AngleAxisd(0.3 * (i + 1), Eigen::Vector3d(i + 1, 2, -1).normalized())
          .toRotationMatrix();
      wrenches.col(i) << 1.0 + i, -2.0 * i, 4.0 - i, 0.3 * i, -0.5, 0.2 + i;
    }
    checkDecomposition(poses, wrenches, length);
    if (count == 1) {
      require(
        decomposeContactWrenches(poses, wrenches, length).internal.norm() < 1e-10,
        "A single full-wrench contact cannot have internal load");
    }
    for (auto & pose : poses) pose.translation().setZero();
    checkDecomposition(poses, wrenches, length);
  }

  Poses poses(4, Eigen::Isometry3d::Identity());
  for (int i = 0; i < 4; ++i) {
    poses[i].translation() = Eigen::Vector3d(i % 2 ? 0.3 : -0.3, i / 2 ? 0.3 : -0.3, -0.2);
  }
  Wrenches wrenches = Wrenches::Zero(6, 4);
  wrenches.row(2).setConstant(4.0);
  require(
    decomposeContactWrenches(poses, wrenches, length).internal.norm() < 1e-10,
    "Symmetric vertical support should have no internal component");
  wrenches.setZero();
  wrenches(0, 0) = 3;
  wrenches(0, 1) = -3;
  require(
    decomposeContactWrenches(poses, wrenches, length).net.norm() < 1e-10,
    "Opposing collinear forces should be purely internal");
  checkDecomposition(poses, wrenches, length);

  const auto rejects = [](const Poses & p, const Wrenches & w, double l) {
    try {
      decomposeContactWrenches(p, w, l);
    } catch (const std::invalid_argument &) {
      return true;
    }
    return false;
  };
  require(rejects({}, Wrenches(6, 0), length), "Empty contact set was accepted");
  require(
    rejects(Poses(3, Eigen::Isometry3d::Identity()), wrenches, length),
    "Size mismatch was accepted");
  for (double invalid :
       {0.0, -1.0, std::numeric_limits<double>::infinity(),
        std::numeric_limits<double>::quiet_NaN()}) {
    require(rejects(poses, wrenches, invalid), "Invalid characteristic length was accepted");
  }
  wrenches(0, 0) = std::numeric_limits<double>::quiet_NaN();
  require(rejects(poses, wrenches, length), "Non-finite wrench was accepted");
  wrenches.setZero();
  auto invalid = poses;
  invalid[0].linear()(0, 0) = -1;
  require(rejects(invalid, wrenches, length), "Reflection was accepted as a rigid pose");
  invalid = poses;
  invalid[0].linear()(0, 0) = 2;
  require(rejects(invalid, wrenches, length), "Non-rigid pose was accepted");
  invalid = poses;
  invalid[0].matrix()(3, 0) = 1;
  require(rejects(invalid, wrenches, length), "Invalid homogeneous row was accepted");
  invalid = poses;
  invalid[0].translation().x() = std::numeric_limits<double>::infinity();
  require(rejects(invalid, wrenches, length), "Non-finite pose was accepted");
  wrenches(3, 0) = std::numeric_limits<double>::max();
  bool overflow_rejected = false;
  try {
    decomposeContactWrenches(poses, wrenches, std::numeric_limits<double>::min());
  } catch (const std::runtime_error &) {
    overflow_rejected = true;
  }
  require(overflow_rejected, "Normalization overflow produced an unchecked result");
  std::cout
    << "Wrench decomposition: 1/3/4 contacts, reconstruction, zero resultant, minimum norm, "
       "frame/unit invariance, invalid inputs and overflow rejection passed\n";
}
