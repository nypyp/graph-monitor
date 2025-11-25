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
project(rosgraph_validator)

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

generate_parameter_library(${PROJECT_NAME}_parameters
                           param/validator_parameters.yaml)

ament_auto_add_library(${PROJECT_NAME}_nodelib SHARED src/validator.cpp)
target_link_libraries(${PROJECT_NAME}_nodelib validator_parameters
                      yaml-cpp::yaml-cpp fmt::fmt)

rclcpp_components_register_node(
  ${PROJECT_NAME}_nodelib PLUGIN "rosgraph_validator::GraphValidator"
  EXECUTABLE ${PROJECT_NAME}_node)

if(BUILD_TESTING)
  ament_auto_find_test_dependencies()
  ament_lint_auto_find_test_dependencies()
endif()

ament_auto_package(INSTALL_TO_SHARE launch config USE_SCOPED_HEADER_INSTALL_DIR)
