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

#include <algorithm>
#include <stdexcept>
#include <tuple>

#include <pinocchio/algorithm/center-of-mass.hpp>
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
}

ComState Kinematics::computeComState(
  const Eigen::VectorXd & q, const Eigen::VectorXd & v, const Eigen::VectorXd & a)
{
  pinocchio::centerOfMass(model_, data_, q, v, a);
  return {data_.com[0], data_.vcom[0], data_.acom[0]};
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

std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> Kinematics::computeBaseManipulability(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  const int k = static_cast<int>(contact_frame_names.size());

  if (k == 0) {
    return {
      0.0, Eigen::VectorXd::Zero(task_dims.size()),
      Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size())};
  }

  // 1. Stacked base and supporting-joint Jacobians: J_b v_b + J_q qdot = 0
  int sub_nv = 0;
  for (const auto & name : joint_names) {
    if (!model_.existJointName(name)) {
      throw std::invalid_argument("Joint '" + name + "' does not exist.");
    }
    sub_nv += model_.joints[model_.getJointId(name)].nv();
  }

  Eigen::MatrixXd J_b = Eigen::MatrixXd::Zero(6 * k, 6);
  Eigen::MatrixXd J_q = Eigen::MatrixXd::Zero(6 * k, sub_nv);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < k; ++i) {
    if (!model_.existFrame(contact_frame_names[i])) {
      throw std::invalid_argument("Frame '" + contact_frame_names[i] + "' does not exist.");
    }

    const pinocchio::FrameIndex frame_id = model_.getFrameId(contact_frame_names[i]);

    pinocchio::Data::Matrix6x J_full(6, model_.nv);
    J_full.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    // Floating-base contribution
    J_b.block(6 * i, 0, 6, 6) = J_full.block(0, 0, 6, 6);

    // Supporting-joint contribution
    int col_offset = 0;
    for (const auto & name : joint_names) {
      const pinocchio::JointIndex j_id = model_.getJointId(name);
      const int nv_i = model_.joints[j_id].nv();
      const int idx_v = model_.joints[j_id].idx_v();

      J_q.block(6 * i, col_offset, 6, nv_i) = J_full.block(0, idx_v, 6, nv_i);
      col_offset += nv_i;
    }
  }

  // 2. SVD of J_b (full U for its left null space); rigid 6-DoF contacts imply rank 6.
  Eigen::JacobiSVD<Eigen::MatrixXd> svd_b(J_b, Eigen::ComputeFullU | Eigen::ComputeFullV);
  const Eigen::VectorXd sigma_b = svd_b.singularValues();
  const double sigma_b_max = (sigma_b.size() > 0) ? sigma_b.maxCoeff() : 0.0;

  // Relative tolerance so round-off residuals are not classified as physical constraints.
  constexpr double rank_rel_tol = 1.0e-10;
  const double tol_b = rank_rel_tol * std::max(1.0, sigma_b_max);

  int rank_b = 0;
  for (int i = 0; i < sigma_b.size(); ++i) {
    if (sigma_b[i] > tol_b) {
      ++rank_b;
    }
  }

  // A unique base velocity cannot be obtained unless J_b has full column rank.
  if (rank_b < 6) {
    return {
      0.0, Eigen::VectorXd::Zero(task_dims.size()),
      Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size())};
  }

  Eigen::MatrixXd Sigma_b_inv = Eigen::MatrixXd::Zero(6, 6);
  for (int i = 0; i < 6; ++i) {
    Sigma_b_inv(i, i) = 1.0 / sigma_b[i];
  }

  // J_b^dagger = V Sigma^-1 U_r^T, with U_r the first six columns of U.
  const Eigen::MatrixXd U_r = svd_b.matrixU().leftCols(6);
  const Eigen::MatrixXd J_b_pinv = svd_b.matrixV() * Sigma_b_inv * U_r.transpose();

  // 3. Closed-chain consistency: U_perp^T J_q qdot = 0, U_perp = left null space of J_b.
  const int left_nullity_b = J_b.rows() - rank_b;

  Eigen::MatrixXd N_c;

  if (left_nullity_b == 0) {
    // Single rigid 6-DoF contact: every supporting-joint velocity is compatible.
    N_c = Eigen::MatrixXd::Identity(sub_nv, sub_nv);
  } else {
    const Eigen::MatrixXd U_perp = svd_b.matrixU().rightCols(left_nullity_b);
    const Eigen::MatrixXd C = U_perp.transpose() * J_q;

    // 4. Orthonormal basis N_c of Null(C): qdot = N_c xidot
    Eigen::JacobiSVD<Eigen::MatrixXd> svd_c(C, Eigen::ComputeFullV);
    const Eigen::VectorXd sigma_c = svd_c.singularValues();
    const double sigma_c_max = (sigma_c.size() > 0) ? sigma_c.maxCoeff() : 0.0;
    const double tol_c = rank_rel_tol * std::max({1.0, sigma_c_max, J_q.norm()});

    int rank_c = 0;
    for (int i = 0; i < sigma_c.size(); ++i) {
      if (sigma_c[i] > tol_c) {
        ++rank_c;
      }
    }

    const int nullity_c = sub_nv - rank_c;
    if (nullity_c <= 0) {
      return {
        0.0, Eigen::VectorXd::Zero(task_dims.size()),
        Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size())};
    }

    N_c = svd_c.matrixV().rightCols(nullity_c);
  }

  // 5. Constraint-consistent equivalent Jacobian: v_b = -J_b^dagger J_q N_c xidot = J_eq xidot
  const Eigen::MatrixXd J_eq = -J_b_pinv * J_q * N_c;

  Eigen::MatrixXd J_eq_task(task_dims.size(), J_eq.cols());
  for (size_t i = 0; i < task_dims.size(); ++i) {
    if (task_dims[i] < 0 || task_dims[i] >= J_eq.rows()) {
      throw std::out_of_range("Invalid task dimension.");
    }
    J_eq_task.row(i) = J_eq.row(task_dims[i]);
  }

  // 6. Manipulability: w = sqrt(det(J_eq J_eq^T))
  const Eigen::MatrixXd J_Jt = J_eq_task * J_eq_task.transpose();

  // Fewer admissible DoF than the evaluated task means zero full-dimensional manipulability.
  double measure = 0.0;
  if (J_eq_task.cols() >= static_cast<int>(task_dims.size())) {
    double determinant = J_Jt.determinant();
    // J_Jt is theoretically PSD; clear only small negative round-off.
    if (determinant < 0.0 && std::abs(determinant) < 1.0e-12) {
      determinant = 0.0;
    }
    measure = (determinant > 0.0) ? std::sqrt(determinant) : 0.0;
  }

  // Manipulability ellipsoid axes.
  Eigen::SelfAdjointEigenSolver<Eigen::MatrixXd> eigensolver(J_Jt);
  Eigen::VectorXd eigenvalues;
  Eigen::MatrixXd eigenvectors;
  if (eigensolver.info() == Eigen::Success) {
    eigenvalues = eigensolver.eigenvalues().cwiseMax(0.0).cwiseSqrt();
    eigenvectors = eigensolver.eigenvectors();
  } else {
    eigenvalues = Eigen::VectorXd::Zero(task_dims.size());
    eigenvectors = Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size());
  }

  return {measure, eigenvalues, eigenvectors};
}

double Kinematics::computeBaseManipulabilityMeasure(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims)
{
  return std::get<0>(computeBaseManipulability(q, contact_frame_names, joint_names, task_dims));
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
