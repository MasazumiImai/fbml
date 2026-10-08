cmake_minimum_required(VERSION 3.8)
project(fbml)

if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the build type" FORCE)
  message(STATUS "fbml: CMAKE_BUILD_TYPE not set, defaulting to Release (-O3).")
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(pinocchio REQUIRED)
find_package(urdfdom REQUIRED)

add_library(${PROJECT_NAME} SHARED
  src/core.cpp
  src/kinematics.cpp
  src/dynamics.cpp
  src/math_utils.cpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

target_link_libraries(${PROJECT_NAME} PUBLIC
  Eigen3::Eigen
  pinocchio::pinocchio
  urdfdom::urdfdom_model
)

ament_export_targets(${PROJECT_NAME}Targets HAS_LIBRARY_TARGET)
ament_export_dependencies(Eigen3 pinocchio urdfdom)

install(DIRECTORY include/
  DESTINATION include
)

install(TARGETS ${PROJECT_NAME}
  EXPORT ${PROJECT_NAME}Targets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

add_compile_definitions(COAL_DISABLE_HPP_FCL_WARNINGS)

include(CTest)
if(BUILD_TESTING)
  add_executable(check_link_inertia test/check_link_inertia.cpp)
  target_link_libraries(check_link_inertia fbml)
  add_test(NAME check_link_inertia COMMAND check_link_inertia)
endif()

ament_package()
