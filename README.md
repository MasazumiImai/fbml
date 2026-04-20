# FBML: Floating-Base Multibody Library

![ROS 2 CI](https://github.com/MasazumiImai/fbml/actions/workflows/ci.yml/badge.svg)

## Requirements

- [ROS 2](https://docs.ros.org/en/humble/index.html) (Humble)
- [Pinocchio](https://github.com/stack-of-tasks/pinocchio)

## Installation

```bash
# Clone repository
mkdir -p ~/fbml/src
cd ~/fbml/src
git clone https://github.com/MasazumiImai/fbml.git

# Build
colcon build --packages-select fbml --symlink-install
source install/setup.bash
```

## Usage

If you are using FBML from other ROS 2 packages, please add the following dependencies to `CMakeLists.txt` and `package.xml`.

#### `CMakeLists.txt`

```CMakeLists.txt
find_package(fbml REQUIRED)

add_executable(your_robot_node src/your_robot_node.cpp)
ament_target_dependencies(your_robot_node fbml)
```

#### `package.xml`

```package.xml
<depend>fbml</depend>
```

#### Example of an include in C++

```cpp
#include <fbml/core.hpp>
#include <fbml/kinematics.hpp>
#include <fbml/dynamics.hpp>

fbml::RobotCore robot("model.urdf");
fbml::Kinematics kin(robot);
```
