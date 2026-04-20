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

#ifndef FBML__DYNAMICS_HPP_
#define FBML__DYNAMICS_HPP_

#include <Eigen/Dense>
#include <string>

#include <pinocchio/multibody/data.hpp>

#include "fbml/core.hpp"
#include "fbml/types.hpp"

namespace fbml
{

class FBML_PUBLIC Dynamics
{
public:
  explicit Dynamics(const RobotCore & core);
  virtual ~Dynamics() = default;

  Eigen::MatrixXd computeMassMatrix(const Eigen::VectorXd & q);

  // ?: Should use struct (PartitionedMassMatrix{fbml::Matrix6d base; Eigen::MatrixXd coupling;}) as output?
  void computePartitionedMassMatrices(
    const Eigen::VectorXd & q, Eigen::MatrixXd & M_b, Eigen::MatrixXd & M_bm);

  Eigen::VectorXd computeNonLinearEffects(const Eigen::VectorXd & q, const Eigen::VectorXd & v);

  Eigen::MatrixXd computeGeneralizedJacobian(
    const Eigen::VectorXd & q, const std::string & frame_name,
    pinocchio::ReferenceFrame reference_frame = pinocchio::LOCAL);

private:
  const pinocchio::Model & model_;
  pinocchio::Data data_;
};

}  // namespace fbml

#endif  // FBML__DYNAMICS_HPP_
