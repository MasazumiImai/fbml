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
: model_(core.getModel()), data_(pinocchio::Data(model_)), core_(core)
{
  const int nv = model_.nv;
  j_ac_ = pinocchio::Data::Matrix6x(6, nv);
  j_ac_.setZero();
  j_sub_ = Eigen::MatrixXd::Zero(6, nv);
  manip_j_task_ = Eigen::MatrixXd::Zero(6, nv);
  bmanip_jeq_task_ = Eigen::MatrixXd::Zero(6, nv);
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

void Kinematics::computeFrameJacobianInto(
  const Eigen::VectorXd & q, const std::string & frame_name,
  pinocchio::ReferenceFrame reference_frame)
{
  if (!model_.existFrame(frame_name)) {
    throw std::invalid_argument("Frame '" + frame_name + "' does not exist in the model.");
  }

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::FrameIndex frame_id = model_.getFrameId(frame_name);
  j_ac_.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, reference_frame, j_ac_);
}

int Kinematics::assembleSubJacobian(const std::vector<std::string> & joint_names)
{
  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    pinocchio::JointIndex j_id = model_.getJointId(name);
    int nv_i = model_.joints[j_id].nv();
    int idx_v = model_.joints[j_id].idx_v();
    j_sub_.middleCols(sub_nv, nv_i) = j_ac_.middleCols(idx_v, nv_i);
    sub_nv += nv_i;
  }
  return sub_nv;
}

double Kinematics::computeManipulabilityCore(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  computeFrameJacobianInto(q, frame_name, pinocchio::LOCAL);
  int sub_nv = assembleSubJacobian(joint_names);
  int t = static_cast<int>(task_dims.size());

  for (int i = 0; i < t; ++i) {
    manip_j_task_.row(i).head(sub_nv) = j_sub_.row(task_dims[i]).head(sub_nv);
  }

  const auto Jt = manip_j_task_.topLeftCorner(t, sub_nv);
  auto JJt = manip_jjt_.topLeftCorner(t, t);
  JJt.noalias() = Jt * Jt.transpose();
  return std::sqrt(std::abs(JJt.determinant()));
}

double Kinematics::computeManipulability(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  return computeManipulabilityCore(q, frame_name, joint_names, task_dims);
}

double Kinematics::computeManipulability(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  Eigen::VectorXd & eigenvalues_out, Eigen::MatrixXd & eigenvectors_out)
{
  double measure = computeManipulabilityCore(q, frame_name, joint_names, task_dims);
  int t = static_cast<int>(task_dims.size());

  auto JJt = manip_jjt_.topLeftCorner(t, t);
  manip_eig_.compute(JJt);
  if (manip_eig_.info() == Eigen::Success) {
    eigenvalues_out = manip_eig_.eigenvalues().cwiseAbs().cwiseSqrt();
    eigenvectors_out = manip_eig_.eigenvectors();
  } else {
    eigenvalues_out = Eigen::VectorXd::Zero(t);
    eigenvectors_out = Eigen::MatrixXd::Identity(t, t);
  }
  return measure;
}

double Kinematics::computeBaseManipulabilityCore(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  int k = static_cast<int>(contact_frame_names.size());
  if (k == 0) {
    return -1.0;  // sentinel: no contact frames
  }

  int t = static_cast<int>(task_dims.size());

  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    sub_nv += model_.joints[model_.getJointId(name)].nv();
  }

  bmanip_jb_.resize(6 * k, 6);
  bmanip_jq_.resize(6 * k, sub_nv);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < k; ++i) {
    if (!model_.existFrame(contact_frame_names[i])) {
      throw std::invalid_argument("Frame '" + contact_frame_names[i] + "' does not exist.");
    }
    pinocchio::FrameIndex frame_id = model_.getFrameId(contact_frame_names[i]);
    j_ac_.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, j_ac_);

    bmanip_jb_.block(6 * i, 0, 6, 6) = j_ac_.block(0, 0, 6, 6);

    int col_offset = 0;
    for (const auto & name : joint_names) {
      pinocchio::JointIndex j_id = model_.getJointId(name);
      int nv_i = model_.joints[j_id].nv();
      int idx_v = model_.joints[j_id].idx_v();
      bmanip_jq_.block(6 * i, col_offset, 6, nv_i) = j_ac_.block(0, idx_v, 6, nv_i);
      col_offset += nv_i;
    }
  }

  // Pseudo-inverse of J_b via thin SVD: J_b^+ = V * Sigma^-1 * U^T
  bmanip_svd_.compute(bmanip_jb_, Eigen::ComputeThinU | Eigen::ComputeThinV);
  const auto & sv = bmanip_svd_.singularValues();
  double tolerance = std::numeric_limits<double>::epsilon() * std::max(6 * k, 6) * std::abs(sv(0));

  Eigen::Matrix<double, 6, 1> sv_inv;
  for (int i = 0; i < sv.size(); ++i) {
    sv_inv(i) = (std::abs(sv(i)) > tolerance) ? (1.0 / sv(i)) : 0.0;
  }

  bmanip_vd_.noalias() = bmanip_svd_.matrixV() * sv_inv.asDiagonal();
  bmanip_pinv_.noalias() = bmanip_vd_ * bmanip_svd_.matrixU().adjoint();
  bmanip_jeq_.noalias() = -bmanip_pinv_ * bmanip_jq_;

  for (int i = 0; i < t; ++i) {
    bmanip_jeq_task_.row(i).head(sub_nv) = bmanip_jeq_.row(task_dims[i]).head(sub_nv);
  }

  const auto Jt = bmanip_jeq_task_.topLeftCorner(t, sub_nv);
  auto JJt = manip_jjt_.topLeftCorner(t, t);
  JJt.noalias() = Jt * Jt.transpose();
  return std::sqrt(std::abs(JJt.determinant()));
}

double Kinematics::computeBaseManipulability(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  double measure = computeBaseManipulabilityCore(q, contact_frame_names, joint_names, task_dims);
  return (measure < 0.0) ? 0.0 : measure;
}

double Kinematics::computeBaseManipulability(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  Eigen::VectorXd & eigenvalues_out, Eigen::MatrixXd & eigenvectors_out)
{
  int t = static_cast<int>(task_dims.size());
  double measure = computeBaseManipulabilityCore(q, contact_frame_names, joint_names, task_dims);

  if (measure < 0.0) {  // no contact frames
    eigenvalues_out = Eigen::VectorXd::Zero(t);
    eigenvectors_out = Eigen::MatrixXd::Identity(t, t);
    return 0.0;
  }

  auto JJt = manip_jjt_.topLeftCorner(t, t);
  manip_eig_.compute(JJt);
  if (manip_eig_.info() == Eigen::Success) {
    eigenvalues_out = manip_eig_.eigenvalues().cwiseAbs().cwiseSqrt();
    eigenvectors_out = manip_eig_.eigenvectors();
  } else {
    eigenvalues_out = Eigen::VectorXd::Zero(t);
    eigenvectors_out = Eigen::MatrixXd::Identity(t, t);
  }
  return measure;
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
      return core_.isWithinJointLimits(q, joint_names);
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

void Kinematics::solveIVK(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const Eigen::Vector<double, 6> & desired_twist_in_local,
  const std::vector<std::string> & joint_names, Eigen::Ref<Eigen::VectorXd> joint_vel_out,
  double damping_factor)
{
  computeFrameJacobianInto(q, frame_name, pinocchio::LOCAL);
  int sub_nv = assembleSubJacobian(joint_names);

  const auto J = j_sub_.leftCols(sub_nv);
  dls_A_.noalias() = J * J.transpose();
  dls_A_.diagonal().array() += damping_factor * damping_factor;

  joint_vel_out.head(sub_nv).noalias() =
    J.transpose() * dls_A_.ldlt().solve(desired_twist_in_local);
}

}  // namespace fbml
