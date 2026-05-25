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
#include <pinocchio/spatial/se3.hpp>

namespace fbml
{
namespace math
{

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

}  // namespace math
}  // namespace fbml
