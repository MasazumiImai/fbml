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

#include "fbml/math_utils.hpp"

#include <pinocchio/spatial/explog.hpp>
#include <pinocchio/spatial/force.hpp>
#include <pinocchio/spatial/motion.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace fbml
{
namespace math
{

Eigen::Vector<double, 6> computePoseError(
  const Eigen::Isometry3d & current_pose, const Eigen::Isometry3d & target_pose)
{
  pinocchio::SE3 cur_se3(current_pose.rotation(), current_pose.translation());
  pinocchio::SE3 tgt_se3(target_pose.rotation(), target_pose.translation());

  // Compute error (T_err = T_cur^-1 * T_tgt)
  pinocchio::SE3 error_se3 = cur_se3.inverse() * tgt_se3;

  // Logarithmic mapping (log6)
  return pinocchio::log6(error_se3).toVector();
}

Eigen::Isometry3d integratePose(
  const Eigen::Isometry3d & current_pose, const Eigen::Vector<double, 6> & twist, double dt)
{
  pinocchio::SE3 cur_se3(current_pose.rotation(), current_pose.translation());

  pinocchio::Motion v_cmd(twist.head<3>(), twist.tail<3>());

  // Integration using the exponential map (T_next = T_cur * exp6(v * dt))
  pinocchio::SE3 next_se3 = cur_se3 * pinocchio::exp6(v_cmd * dt);

  Eigen::Isometry3d result = Eigen::Isometry3d::Identity();
  result.linear() = next_se3.rotation();
  result.translation() = next_se3.translation();

  return result;
}

Eigen::Isometry3d transformPose(
  const Eigen::Isometry3d & pose_in_local, const Eigen::Isometry3d & pose_local_in_ref)
{
  pinocchio::SE3 se3_in_local(pose_in_local.rotation(), pose_in_local.translation());
  pinocchio::SE3 se3_local_in_ref(pose_local_in_ref.rotation(), pose_local_in_ref.translation());

  pinocchio::SE3 se3_in_ref = se3_local_in_ref.act(se3_in_local);

  Eigen::Isometry3d pose_in_ref = Eigen::Isometry3d::Identity();
  pose_in_ref.linear() = se3_in_ref.rotation();
  pose_in_ref.translation() = se3_in_ref.translation();

  return pose_in_ref;
}

Eigen::Vector<double, 6> transformTwist(
  const Eigen::Vector<double, 6> & twist_in_local, const Eigen::Isometry3d & pose_local_in_ref)
{
  pinocchio::SE3 se3_local_in_ref(pose_local_in_ref.linear(), pose_local_in_ref.translation());
  pinocchio::Motion motion_in_local(twist_in_local.head<3>(), twist_in_local.tail<3>());

  pinocchio::Motion motion_in_ref = se3_local_in_ref.act(motion_in_local);

  return motion_in_ref.toVector();
}

Eigen::Vector<double, 6> transformWrench(
  const Eigen::Vector<double, 6> & wrench_in_local, const Eigen::Isometry3d & pose_local_in_ref)
{
  pinocchio::SE3 se3_local_in_ref(pose_local_in_ref.linear(), pose_local_in_ref.translation());
  pinocchio::Force force_in_local(wrench_in_local.head<3>(), wrench_in_local.tail<3>());

  pinocchio::Force force_in_ref = se3_local_in_ref.act(force_in_local);

  return force_in_ref.toVector();
}

}  // namespace math
}  // namespace fbml
