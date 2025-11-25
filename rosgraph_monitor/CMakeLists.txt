# Copyright 2024, Bonsai Robotics, Inc - All Rights Reserved
#
# Licensed under the Apache License, Version 2.0 (the "License"); you may not
# use this file except in compliance with the License. You may obtain a copy of
# the License at
#
# http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS, WITHOUT
# WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied. See the
# License for the specific language governing permissions and limitations under
# the License.

cmake_minimum_required(VERSION 3.8)
project(rosgraph_monitor)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic -Werror=switch)
  add_link_options("-Wl,--no-undefined")
endif()

find_package(yaml-cpp REQUIRED)
find_package(fmt REQUIRED)
find_package(ament_cmake_auto REQUIRED)
ament_auto_find_build_dependencies()

if(NOT DEFINED ENV{ROS_DISTRO})
  message(FATAL_ERROR "ROS_DISTRO environment variable not set")
else()
  string(TOUPPER "$ENV{ROS_DISTRO}" ROS_DISTRO)
  message(STATUS "Detected ROS 2 distro ${ROS_DISTRO} from environment")
  add_compile_definitions("ROS2_${ROS_DISTRO}")
endif()

# Library
ament_auto_add_library(${PROJECT_NAME} SHARED src/event.cpp src/monitor.cpp)

generate_parameter_library(${PROJECT_NAME}_generated_parameters
                           src/params_decl.yaml)

generate_parameter_library(validator_parameters param/validator_parameters.yaml)

# Node
ament_auto_add_library(${PROJECT_NAME}_nodelib SHARED src/node.cpp)
target_link_libraries(${PROJECT_NAME}_nodelib
                      ${PROJECT_NAME}_generated_parameters)

ament_auto_add_library(${PROJECT_NAME}_cumulatorlib SHARED src/cumulator.cpp)

ament_auto_add_library(${PROJECT_NAME}_validatorlib SHARED src/validator.cpp)
target_link_libraries(${PROJECT_NAME}_validatorlib validator_parameters
                      yaml-cpp::yaml-cpp fmt::fmt)

rclcpp_components_register_node(
  ${PROJECT_NAME}_nodelib PLUGIN "rosgraph_monitor::Node" EXECUTABLE
  ${PROJECT_NAME}_node)

rclcpp_components_register_node(
  ${PROJECT_NAME}_cumulatorlib PLUGIN "rosgraph_monitor::GraphCumulator"
  EXECUTABLE ${PROJECT_NAME}_cumulator)

rclcpp_components_register_node(
  ${PROJECT_NAME}_validatorlib PLUGIN "rosgraph_monitor::GraphValidator"
  EXECUTABLE ${PROJECT_NAME}_validator)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()

  ament_add_gmock(test_graph_monitor test/test_graph_monitor.cpp)
  target_link_libraries(test_graph_monitor ${PROJECT_NAME})
endif()

ament_auto_package(INSTALL_TO_SHARE launch config USE_SCOPED_HEADER_INSTALL_DIR)
