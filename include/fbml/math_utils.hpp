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
 * @brief Transform pose from frame A to B
 *
 * @param pose_in_A Pose in frame A
 * @param pose_A_to_B Pose of frame B relative to frame A
 * @return Eigen::Isometry3d Pose in frame B
 */
FBML_PUBLIC
Eigen::Isometry3d transformPose(
  const Eigen::Isometry3d & pose_in_A, const Eigen::Isometry3d & pose_A_to_B);

/**
 * @brief Transform twist (spatial velocity) from frame A to B
 *
 * @param twist_in_A Twist in frame A [vx, vy, vz, wx, wy, wz]^T
 * @param pose_A_to_B Pose of frame B from frame A
 * @return Eigen::Vector<double, 6> Twist in frame B
 */
FBML_PUBLIC
Eigen::Vector<double, 6> transformTwist(
  const Eigen::Vector<double, 6> & twist_in_A, const Eigen::Isometry3d & pose_A_to_B);

/**
 * @brief Transform wrench (spatial force) from frame A to B
 *
 * @param wrench_in_A Wrench in A [fx, fy, fz, nx, ny, nz]^T
 * @param pose_A_to_B Pose of frame B from frame A
 * @return Eigen::Vector<double, 6> Wrench in B
 */
FBML_PUBLIC
Eigen::Vector<double, 6> transformWrench(
  const Eigen::Vector<double, 6> & wrench_in_A, const Eigen::Isometry3d & pose_A_to_B);

}  // namespace math
}  // namespace fbml

#endif  // FBML__MATH_UTILS_HPP_
