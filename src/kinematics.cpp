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

#include "fbml/kinematics.hpp"

#include <stdexcept>

#include <pinocchio/algorithm/frames.hpp>
#include <pinocchio/algorithm/jacobian.hpp>
#include <pinocchio/algorithm/joint-configuration.hpp>
#include <pinocchio/algorithm/kinematics.hpp>
#include <pinocchio/spatial/se3.hpp>

namespace fbml
{

Kinematics::Kinematics(const RobotCore & core)
: model_(core.getModel()), data_(pinocchio::Data(model_))
{
}

Eigen::MatrixXd Kinematics::computeJacobian(
  const Eigen::VectorXd & q, const std::string & frame_name,
  pinocchio::ReferenceFrame reference_frame)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist in the model.");
  }

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  pinocchio::Data::Matrix6x J(6, model_.nv);
  J.setZero();

  pinocchio::getFrameJacobian(model_, data_, frame_id, reference_frame, J);

  return J;
}

std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> Kinematics::computeManipulability(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  Eigen::MatrixXd J_full = computeJacobian(q, frame_name, pinocchio::LOCAL);

  int sub_nv = 0;
  for (const auto & name : joint_names) {
    sub_nv += model_.joints[model_.getJointId(name)].nv();
  }

  Eigen::MatrixXd J_sub(6, sub_nv);
  int col_offset = 0;
  for (const auto & name : joint_names) {
    pinocchio::JointIndex j_id = model_.getJointId(name);
    int nv_i = model_.joints[j_id].nv();
    int idx_v = model_.joints[j_id].idx_v();
    J_sub.middleCols(col_offset, nv_i) = J_full.middleCols(idx_v, nv_i);
    col_offset += nv_i;
  }

  Eigen::MatrixXd J_task(task_dims.size(), sub_nv);
  for (size_t i = 0; i < task_dims.size(); ++i) {
    J_task.row(i) = J_sub.row(task_dims[i]);
  }

  Eigen::MatrixXd J_Jt = J_task * J_task.transpose();
  double measure = std::sqrt(std::abs(J_Jt.determinant()));

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(J_Jt);
  Eigen::VectorXd eigenvalues;
  Eigen::MatrixXd eigenvectors;

  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues().cwiseAbs().cwiseSqrt();
    eigenvectors = eigensolver.eigenvectors();
  } else {
    eigenvalues = Eigen::VectorXd::Zero(task_dims.size());
    eigenvectors = Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size());
  }

  return {measure, eigenvalues, eigenvectors};
}

std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> Kinematics::computeBaseManipulability(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  int k = contact_frame_names.size();
  if (k == 0) {
    return {
      0.0, Eigen::VectorXd::Zero(task_dims.size()),
      Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size())};
  }

  int sub_nv = 0;
  for (const auto & name : joint_names) {
    sub_nv += model_.joints[model_.getJointId(name)].nv();
  }

  Eigen::MatrixXd J_b(6 * k, 6);
  Eigen::MatrixXd J_q(6 * k, sub_nv);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < k; ++i) {
    pinocchio::FrameIndex frame_id = model_.getFrameId(contact_frame_names[i]);

    pinocchio::Data::Matrix6x J_full(6, model_.nv);
    J_full.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    J_b.block(6 * i, 0, 6, 6) = J_full.block(0, 0, 6, 6);

    int col_offset = 0;
    for (const auto & name : joint_names) {
      pinocchio::JointIndex j_id = model_.getJointId(name);
      int nv_i = model_.joints[j_id].nv();
      int idx_v = model_.joints[j_id].idx_v();
      J_q.block(6 * i, col_offset, 6, nv_i) = J_full.block(0, idx_v, 6, nv_i);
      col_offset += nv_i;
    }
  }

  Eigen::JacobiSVD<Eigen::MatrixXd> svd(J_b, Eigen::ComputeThinU | Eigen::ComputeThinV);
  double tolerance = std::numeric_limits<double>::epsilon() * std::max(J_b.cols(), J_b.rows()) *
    svd.singularValues().array().abs()(0);

  Eigen::MatrixXd J_b_pinv = svd.matrixV() *
    (svd.singularValues().array().abs() > tolerance)
      .select(svd.singularValues().array().inverse(), 0)
      .matrix()
      .asDiagonal() *
    svd.matrixU().adjoint();

  Eigen::MatrixXd J_eq = -J_b_pinv * J_q;

  Eigen::MatrixXd J_eq_task(task_dims.size(), sub_nv);
  for (size_t i = 0; i < task_dims.size(); ++i) {
    J_eq_task.row(i) = J_eq.row(task_dims[i]);
  }

  Eigen::MatrixXd J_Jt = J_eq_task * J_eq_task.transpose();
  double measure = std::sqrt(std::abs(J_Jt.determinant()));

  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(J_Jt);
  Eigen::VectorXd eigenvalues;
  Eigen::MatrixXd eigenvectors;

  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues().cwiseAbs().cwiseSqrt();
    eigenvectors = eigensolver.eigenvectors();
  } else {
    eigenvalues = Eigen::VectorXd::Zero(task_dims.size());
    eigenvectors = Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size());
  }

  return {measure, eigenvalues, eigenvectors};
}

Eigen::Isometry3d Kinematics::solveFK(
  const Eigen::VectorXd & q, const std::string & target_frame, const std::string & reference_frame)
{
  if (!model_.existFrame(target_frame)) {
    throw std::invalid_argument("Frame '" + target_frame + "' does not exist in the model.");
  }

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::FrameIndex target_id = model_.getFrameId(target_frame);
  pinocchio::SE3 pose_se3;

  if (reference_frame == "world") {
    pose_se3 = data_.oMf[target_id];
  } else {
    if (!model_.existFrame(reference_frame)) {
      throw std::invalid_argument("Frame '" + reference_frame + "' does not exist in the model.");
    }
    pinocchio::FrameIndex ref_id = model_.getFrameId(reference_frame);
    pose_se3 = data_.oMf[ref_id].actInv(data_.oMf[target_id]);
  }

  Eigen::Isometry3d pose = Eigen::Isometry3d::Identity();
  pose.linear() = pose_se3.rotation();
  pose.translation() = pose_se3.translation();

  return pose;
}

bool Kinematics::solveNumericalIK(
  Eigen::VectorXd & q, const std::string & frame_name, const Eigen::Isometry3d & desired_pose,
  const std::vector<std::string> & joint_names, const std::string & reference_frame,
  const IKSettings & settings)
{
  // TODO: Add kinematic feasibility (joint limits) check (Stop if a joint seems like it’s about to exceed its limit)

  pinocchio::SE3 des_pose_se3(desired_pose.rotation(), desired_pose.translation());

  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist.");
  }
  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);

  pinocchio::FrameIndex ref_id = 0;
  bool use_ref = (reference_frame != "world");
  if (use_ref) {
    if (!model_.existFrame(reference_frame)) {
      throw std::invalid_argument("Frame '" + reference_frame + "' does not exist.");
    }
    ref_id = model_.getFrameId(reference_frame);
  }

  std::vector<pinocchio::JointIndex> joint_ids;
  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    pinocchio::JointIndex j_id = model_.getJointId(name);
    joint_ids.push_back(j_id);
    sub_nv += model_.joints[j_id].nv();
  }

  pinocchio::Data::Matrix6x J_sub(6, sub_nv);
  pinocchio::Data::Matrix6x J_full(6, model_.nv);

  for (int iter = 0; iter < settings.max_iterations; ++iter) {
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    pinocchio::SE3 des_pose_in_world = des_pose_se3;
    if (use_ref) {
      // _oM_ref * _refM_des = _oM_des
      des_pose_in_world = data_.oMf[ref_id] * des_pose_se3;
    }

    pinocchio::SE3 T_cur = data_.oMf[frame_id];
    pinocchio::SE3 T_err = T_cur.inverse() * des_pose_in_world;
    Eigen::VectorXd error = pinocchio::log6(T_err).toVector();

    error = settings.task_weights.cwiseProduct(error);

    if (error.norm() < settings.tolerance) {
      return true;
    }

    J_full.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, J_full);

    int col_offset = 0;
    for (const auto & j_id : joint_ids) {
      int nv_i = model_.joints[j_id].nv();
      int idx_v = model_.joints[j_id].idx_v();
      J_sub.middleCols(col_offset, nv_i) = J_full.middleCols(idx_v, nv_i);
      col_offset += nv_i;
    }

    J_sub = settings.task_weights.asDiagonal() * J_sub;

    // Damped Least Squares (DLS)

    // A = J * J^T + lambda * I
    Eigen::MatrixXd A = J_sub * J_sub.transpose();
    A.diagonal().array() += settings.damping_factor;

    // dq_sub = J^T * A^-1 * error
    Eigen::VectorXd dq_sub = J_sub.transpose() * A.ldlt().solve(error);

    Eigen::VectorXd v_full = Eigen::VectorXd::Zero(model_.nv);
    col_offset = 0;
    for (const auto & j_id : joint_ids) {
      int nv_i = model_.joints[j_id].nv();
      int idx_v = model_.joints[j_id].idx_v();
      v_full.segment(idx_v, nv_i) = dq_sub.segment(col_offset, nv_i);
      col_offset += nv_i;
    }

    q = pinocchio::integrate(model_, q, v_full);
  }

  return false;
}

Eigen::VectorXd Kinematics::solveIVK(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const Eigen::Vector<double, 6> & desired_twist_in_local,
  const std::vector<std::string> & joint_names, double damping_factor)
{
  Eigen::MatrixXd J_full = computeJacobian(q, frame_name, pinocchio::LOCAL);

  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    sub_nv += model_.joints[model_.getJointId(name)].nv();
  }

  Eigen::MatrixXd J_sub(6, sub_nv);
  int col_offset = 0;
  for (const auto & name : joint_names) {
    pinocchio::JointIndex j_id = model_.getJointId(name);
    int nv_i = model_.joints[j_id].nv();
    int idx_v = model_.joints[j_id].idx_v();

    J_sub.middleCols(col_offset, nv_i) = J_full.middleCols(idx_v, nv_i);
    col_offset += nv_i;
  }

  Eigen::MatrixXd J_Jt = J_sub * J_sub.transpose();
  Eigen::MatrixXd A = J_Jt + (damping_factor * damping_factor) * Eigen::MatrixXd::Identity(6, 6);

  Eigen::VectorXd joint_vel = J_sub.transpose() * A.ldlt().solve(desired_twist_in_local);

  return joint_vel;
}

}  // namespace fbml
