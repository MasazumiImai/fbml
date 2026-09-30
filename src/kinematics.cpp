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
  joint_ids_.reserve(static_cast<std::size_t>(model_.njoints));
  ik_dq_ = Eigen::VectorXd::Zero(nv);
  ik_v_ = Eigen::VectorXd::Zero(nv);
  ik_q_next_ = Eigen::VectorXd::Zero(model_.nq);
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
  const pinocchio::FrameIndex frame_id = core_.frameId(frame_name);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::Data::Matrix6x J(6, model_.nv);
  J.setZero();

  pinocchio::getFrameJacobian(model_, data_, frame_id, reference_frame, J);

  return J;
}

void Kinematics::computeFrameJacobianInto(
  const Eigen::VectorXd & q, const std::string & frame_name,
  pinocchio::ReferenceFrame reference_frame)
{
  const pinocchio::FrameIndex frame_id = core_.frameId(frame_name);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  j_ac_.setZero();
  pinocchio::getFrameJacobian(model_, data_, frame_id, reference_frame, j_ac_);
}

int Kinematics::resolveJoints(const std::vector<std::string> & joint_names)
{
  joint_ids_.clear();
  int sub_nv = 0;
  for (const auto & name : joint_names) {
    const pinocchio::JointIndex j_id = core_.jointId(name);
    if (std::find(joint_ids_.begin(), joint_ids_.end(), j_id) != joint_ids_.end()) {
      throw std::invalid_argument("Joint '" + name + "' is listed more than once.");
    }
    joint_ids_.push_back(j_id);
    sub_nv += model_.joints[j_id].nv();
  }
  return sub_nv;
}

void Kinematics::validateTaskDims(const std::vector<int> & task_dims)
{
  if (task_dims.empty() || task_dims.size() > 6) {
    throw std::invalid_argument("task_dims must contain between 1 and 6 entries.");
  }
  for (std::size_t i = 0; i < task_dims.size(); ++i) {
    if (
      task_dims[i] < 0 || task_dims[i] >= 6 ||
      std::find(task_dims.begin(), task_dims.begin() + i, task_dims[i]) != task_dims.begin() + i) {
      throw std::invalid_argument("task_dims must be distinct indices in [0, 6).");
    }
  }
}

void Kinematics::assembleSubJacobian()
{
  int col_offset = 0;
  for (const auto & j_id : joint_ids_) {
    const int nv_i = model_.joints[j_id].nv();
    j_sub_.middleCols(col_offset, nv_i) = j_ac_.middleCols(model_.joints[j_id].idx_v(), nv_i);
    col_offset += nv_i;
  }
}

double Kinematics::computeManipulabilityMeasure(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  double characteristic_length)
{
  if (characteristic_length <= 0.0) {
    throw std::invalid_argument("Characteristic length must be positive.");
  }
  validateTaskDims(task_dims);
  const int sub_nv = resolveJoints(joint_names);

  computeFrameJacobianInto(q, frame_name, pinocchio::LOCAL);
  assembleSubJacobian();
  const int t = static_cast<int>(task_dims.size());

  for (int i = 0; i < t; ++i) {
    manip_j_task_.row(i).head(sub_nv) = j_sub_.row(task_dims[i]).head(sub_nv);
    // Spatial velocity ordering is [linear, angular]; scale only translational rows by 1/l_c.
    if (task_dims[i] < 3) {
      manip_j_task_.row(i).head(sub_nv) /= characteristic_length;
    }
  }

  const auto Jt = manip_j_task_.topLeftCorner(t, sub_nv);
  auto JJt = manip_jjt_.topLeftCorner(t, t);
  JJt.noalias() = Jt * Jt.transpose();
  return std::sqrt(std::abs(JJt.determinant()));
}

std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> Kinematics::computeManipulability(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  double characteristic_length)
{
  const double measure =
    computeManipulabilityMeasure(q, frame_name, joint_names, task_dims, characteristic_length);
  const int t = static_cast<int>(task_dims.size());

  manip_eig_.compute(manip_jjt_.topLeftCorner(t, t));
  if (manip_eig_.info() != Eigen::Success) {
    return {measure, Eigen::VectorXd::Zero(t), Eigen::MatrixXd::Identity(t, t)};
  }
  return {measure, manip_eig_.eigenvalues().cwiseMax(0.0).cwiseSqrt(), manip_eig_.eigenvectors()};
}

std::tuple<double, Eigen::VectorXd, Eigen::MatrixXd> Kinematics::computeBaseManipulability(
  const Eigen::VectorXd & q, const std::vector<std::string> & contact_frame_names,
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  double characteristic_length)
{
  if (characteristic_length <= 0.0) {
    throw std::invalid_argument("Characteristic length must be positive.");
  }
  validateTaskDims(task_dims);
  const int sub_nv = resolveJoints(joint_names);

  const int k = static_cast<int>(contact_frame_names.size());

  if (k == 0) {
    return {
      0.0, Eigen::VectorXd::Zero(task_dims.size()),
      Eigen::MatrixXd::Identity(task_dims.size(), task_dims.size())};
  }

  // 1. Stacked base and supporting-joint Jacobians: J_b v_b + J_q qdot = 0
  Eigen::MatrixXd J_b = Eigen::MatrixXd::Zero(6 * k, 6);
  Eigen::MatrixXd J_q = Eigen::MatrixXd::Zero(6 * k, sub_nv);

  pinocchio::computeJointJacobians(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  for (int i = 0; i < k; ++i) {
    const pinocchio::FrameIndex frame_id = core_.frameId(contact_frame_names[i]);

    pinocchio::Data::Matrix6x J_full(6, model_.nv);
    J_full.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL_WORLD_ALIGNED, J_full);

    // Floating-base contribution
    J_b.block(6 * i, 0, 6, 6) = J_full.block(0, 0, 6, 6);

    // Supporting-joint contribution
    int col_offset = 0;
    for (const auto & j_id : joint_ids_) {
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
    J_eq_task.row(i) = J_eq.row(task_dims[i]);
    // Spatial velocity ordering is [linear, angular]; scale only translational rows by 1/l_c.
    if (task_dims[i] < 3) {
      J_eq_task.row(i) /= characteristic_length;
    }
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
  const std::vector<std::string> & joint_names, const std::vector<int> & task_dims,
  double characteristic_length)
{
  return std::get<0>(computeBaseManipulability(
    q, contact_frame_names, joint_names, task_dims, characteristic_length));
}

Eigen::Isometry3d Kinematics::solveFK(
  const Eigen::VectorXd & q, const std::string & target_frame, const std::string & reference_frame)
{
  const pinocchio::FrameIndex target_id = core_.frameId(target_frame);

  pinocchio::forwardKinematics(model_, data_, q);
  pinocchio::updateFramePlacements(model_, data_);

  pinocchio::SE3 pose_se3;

  if (reference_frame == "world") {
    pose_se3 = data_.oMf[target_id];
  } else {
    pose_se3 = data_.oMf[core_.frameId(reference_frame)].actInv(data_.oMf[target_id]);
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

  const pinocchio::FrameIndex frame_id = core_.frameId(frame_name);
  const bool use_ref = (reference_frame != "world");
  const pinocchio::FrameIndex ref_id = use_ref ? core_.frameId(reference_frame) : 0;

  // Preallocated workspaces keep this loop heap-free for real-time callers.
  const int sub_nv = resolveJoints(joint_names);

  auto J_sub = j_sub_.leftCols(sub_nv);
  auto dq_sub = ik_dq_.head(sub_nv);

  for (int iter = 0; iter < settings.max_iterations; ++iter) {
    pinocchio::computeJointJacobians(model_, data_, q);
    pinocchio::updateFramePlacements(model_, data_);

    // _oM_ref * _refM_des = _oM_des
    const pinocchio::SE3 des_pose_in_world =
      use_ref ? data_.oMf[ref_id] * des_pose_se3 : des_pose_se3;
    const Eigen::Vector<double, 6> error = settings.task_weights.cwiseProduct(
      pinocchio::log6(data_.oMf[frame_id].actInv(des_pose_in_world)).toVector());

    if (error.norm() < settings.tolerance) {
      return core_.isWithinJointLimits(q, joint_names);
    }

    j_ac_.setZero();
    pinocchio::getFrameJacobian(model_, data_, frame_id, pinocchio::LOCAL, j_ac_);
    assembleSubJacobian();
    J_sub.array().colwise() *= settings.task_weights.array();

    // Damped Least Squares: dq = J^T (J J^T + lambda I)^-1 e
    dls_A_.noalias() = J_sub * J_sub.transpose();
    dls_A_.diagonal().array() += settings.damping_factor;
    dq_sub.noalias() = J_sub.transpose() * dls_A_.ldlt().solve(error);

    ik_v_.setZero();
    int col_offset = 0;
    for (const auto & j_id : joint_ids_) {
      int nv_i = model_.joints[j_id].nv();
      int idx_v = model_.joints[j_id].idx_v();
      ik_v_.segment(idx_v, nv_i) = dq_sub.segment(col_offset, nv_i);
      col_offset += nv_i;
    }

    pinocchio::integrate(model_, q, ik_v_, ik_q_next_);
    q = ik_q_next_;
  }

  return false;
}

void Kinematics::solveIVK(
  const Eigen::VectorXd & q, const std::string & frame_name,
  const Eigen::Vector<double, 6> & desired_twist_in_local,
  const std::vector<std::string> & joint_names, Eigen::Ref<Eigen::VectorXd> joint_vel_out,
  double damping_factor)
{
  const int sub_nv = resolveJoints(joint_names);
  if (joint_vel_out.size() < sub_nv) {
    throw std::invalid_argument("joint_vel_out is smaller than the joints' velocity dimension.");
  }
  computeFrameJacobianInto(q, frame_name, pinocchio::LOCAL);
  assembleSubJacobian();

  const auto J = j_sub_.leftCols(sub_nv);
  dls_A_.noalias() = J * J.transpose();
  dls_A_.diagonal().array() += damping_factor * damping_factor;

  joint_vel_out.head(sub_nv).noalias() =
    J.transpose() * dls_A_.ldlt().solve(desired_twist_in_local);
}

}  // namespace fbml
