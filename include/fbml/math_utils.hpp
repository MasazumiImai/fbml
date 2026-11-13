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

#ifndef FBML__MATH_UTILS_HPP_
#define FBML__MATH_UTILS_HPP_

#include <Eigen/Dense>
#include <vector>

#include "fbml/visibility_control.h"

namespace fbml
{
namespace math
{

/**
 * @brief Compute the error between two poses (equivalent to the logarithmic mapping in SE3 / log6)
 *
 * @param current_pose Current pose
 * @param target_pose Target pose
 * @return Eigen::Vector<double, 6> Pose error in local frame [vx, vy, vz, wx, wy, wz]^T
 */
FBML_PUBLIC
Eigen::Vector<double, 6> computePoseError(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & target_pose);

/**
 * @brief Integrate the twist with the current pose to compute the pose for the next step
 *
 * @param current_pose Current pose
 * @param twist Twist in local frame [vx, vy, vz, wx, wy, wz]^T
 * @param dt Step time [s]
 * @return Eigen::Isometry3d Next pose after integration
 */
FBML_PUBLIC
Eigen::Isometry3d integratePose(
  const Eigen::Isometry3d & current_pose, const Eigen::Vector<double, 6> & twist, double dt);

/**
 * @brief Transform pose from a local frame to a reference frame (Forward transformation).
 *
 * @param pose_in_local Pose expressed in the local frame.
 * @param pose_local_in_ref Pose of the local frame viewed from the reference frame.
 * @return Eigen::Isometry3d Pose expressed in the reference frame.
 */
FBML_PUBLIC
Eigen::Isometry3d transformPose(
  const Eigen::Isometry3d & pose_in_local, const Eigen::Isometry3d & pose_local_in_ref);

/**
 * @brief Transform twist (spatial velocity) from a local frame to a reference frame (Forward transformation).
 *
 * @param twist_in_local Twist in the local frame [vx, vy, vz, wx, wy, wz]^T.
 * @param pose_local_in_ref Pose of the local frame viewed from the reference frame.
 * @return Eigen::Vector<double, 6> Twist expressed in the reference frame.
 */
FBML_PUBLIC
Eigen::Vector<double, 6> transformTwist(
  const Eigen::Vector<double, 6> & twist_in_local, const Eigen::Isometry3d & pose_local_in_ref);

/**
 * @brief Transform wrench (spatial force) from a local frame to a reference frame (Forward transformation).
 *
 * @param wrench_in_local Wrench in the local frame [fx, fy, fz, nx, ny, nz]^T.
 * @param pose_local_in_ref Pose of the local frame viewed from the reference frame.
 * @return Eigen::Vector<double, 6> Wrench expressed in the reference frame.
 */
FBML_PUBLIC
Eigen::Vector<double, 6> transformWrench(
  const Eigen::Vector<double, 6> & wrench_in_local, const Eigen::Isometry3d & pose_local_in_ref);

struct ContactWrenchDecomposition
{
  Eigen::Vector3d reference_point;  // Contact-origin centroid in the reference frame.
  Eigen::Vector<double, 6> resultant;  // In reference axes, about reference_point.
  // One column per contact, in its own frame and about its own origin, in physical units.
  Eigen::Matrix<double, 6, Eigen::Dynamic> net;
  Eigen::Matrix<double, 6, Eigen::Dynamic> internal;
};

// Reuses working storage for a fixed contact count. Returned data lives until the next compute.
class FBML_PUBLIC ContactWrenchDecomposer
{
public:
  explicit ContactWrenchDecomposer(Eigen::Index contact_count);
  const ContactWrenchDecomposition & compute(
    const std::vector<Eigen::Isometry3d> & contact_poses,
    const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> & contact_wrenches,
    double characteristic_length);

private:
  ContactWrenchDecomposition result_;
  Eigen::Matrix<double, 6, Eigen::Dynamic> grasp_, normalized_;
  Eigen::MatrixXd square_grasp_;
  Eigen::JacobiSVD<Eigen::MatrixXd, Eigen::NoQRPreconditioner> svd_;
  Eigen::VectorXd net_scaled_;
};

/**
 * @brief Split contact wrenches into minimum-norm support and zero-resultant components.
 * @note Allocates working matrices; not suitable for a hard real-time loop.
 *
 * @param contact_poses Contact frames expressed in one common reference frame.
 * @param contact_wrenches One [force; moment] column per contact, in that contact's frame.
 * @param characteristic_length Positive finite length used to normalize moments to forces.
 * @return net minimizes sum(|force|^2 + |moment/length|^2) while preserving the resultant;
 *         internal = contact_wrenches - net. Neither component enforces contact/actuator limits.
 *         The resultant is about the contact-origin centroid. Every contact is modeled as
 *         transmitting a full six-axis wrench, regardless of the limb's actuation.
 */
FBML_PUBLIC
ContactWrenchDecomposition decomposeContactWrenches(
  const std::vector<Eigen::Isometry3d> & contact_poses,
  const Eigen::Ref<const Eigen::Matrix<double, 6, Eigen::Dynamic>> & contact_wrenches,
  double characteristic_length);

}  // namespace math
}  // namespace fbml

#endif  // FBML__MATH_UTILS_HPP_
