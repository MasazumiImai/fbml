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

// Link inertia must survive fixed-joint merging without changing the dynamics model.
#include <chrono>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <stdexcept>

#include <pinocchio/parsers/urdf.hpp>

#include "fbml/core.hpp"

void require(bool condition, const char * message)
{
  if (!condition) throw std::runtime_error(message);
}

int main()
{
  const auto path = std::filesystem::temp_directory_path() /
    ("fbml_link_inertia_" +
     std::to_string(std::chrono::steady_clock::now().time_since_epoch().count()) + ".urdf");
  {
    std::ofstream file(path);
    file << R"(<robot name="link_inertia">
      <link name="base"><inertial><mass value="2"/>
        <inertia ixx="1" iyy="1" izz="1" ixy="0" ixz="0" iyz="0"/>
      </inertial></link>
      <joint name="tool" type="fixed"><parent link="base"/><child link="tool"/>
        <origin xyz="0.4 0.2 -0.1" rpy="0.1 0.2 -0.3"/>
      </joint>
      <link name="tool"><inertial>
        <origin xyz="0.1 -0.2 0.3" rpy="0 0 1.5707963267948966"/><mass value="3"/>
        <inertia ixx="1" iyy="2" izz="3" ixy="0.2" ixz="0.1" iyz="0.3"/>
      </inertial></link>
      <joint name="marker_joint" type="fixed"><parent link="tool"/><child link="marker"/></joint>
      <link name="marker"/>
    </robot>)";
    file.close();
    require(bool(file), "Could not write test URDF");
  }
  fbml::RobotCore core(path.string());
  pinocchio::Model expected;
  pinocchio::urdf::buildModel(path.string(), pinocchio::JointModelFreeFlyer(), expected);
  std::filesystem::remove(path);
  require(std::abs(core.getTotalMass() - 5.0) < 1e-12, "Fixed-link mass was double-counted");
  require(
    core.getModel().nv == expected.nv && core.getModel().nq == expected.nq,
    "Dynamics dimensions changed");
  for (std::size_t i = 0; i < expected.inertias.size(); ++i) {
    require(
      core.getModel().inertias[i].isApprox(expected.inertias[i], 1e-12), "Joint inertia changed");
  }
  const auto & tool = core.linkInertia("tool");
  require(tool.mass() == 3.0, "Returned combined or joint-frame inertia");
  require(tool.lever().isApprox(Eigen::Vector3d(0.1, -0.2, 0.3), 1e-12), "CoM is not in link axes");
  Eigen::Matrix3d tensor;
  tensor << 2, -0.2, -0.3, -0.2, 1, 0.1, -0.3, 0.1, 3;
  require(tool.inertia().matrix().isApprox(tensor, 1e-12), "Inertial-origin rotation was lost");
  require(core.linkInertia("marker").isZero(), "Missing inertial must produce zero inertia");
  bool rejected = false;
  try {
    core.linkInertia("marker_joint");  // Existing joint frame, but not a link.
  } catch (const std::invalid_argument &) {
    rejected = true;
  }
  require(rejected, "Non-link name was accepted");
  std::cout << "Link inertia: fixed merge, CoM, rotated tensor and name validation passed\n";
}
