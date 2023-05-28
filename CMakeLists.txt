# This file is the cmake file of the LEGO assembly from human demonstration project.
# Copyright (C) 2023

# Authors:
# Ruixuan Liu: ruixuanl@andrew.cmu.edu
# Changliu Liu : cliu6@andrew.cmu.edu

# This program is free software; you can redistribute it and/or
# modify it under the terms of the GNU General Public License
# as published by the Free Software Foundation; either version 3
# of the License, or (at your option) any later version.
 
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.

# You should have received a copy of the GNU General Public License
# along with this program.  If not, see <https://www.gnu.org/licenses/>.

cmake_minimum_required(VERSION 3.10 FATAL_ERROR)
set(CMAKE_CXX_STANDARD 14)
project(lego_assembly VERSION 1.0.0 LANGUAGES CXX)

# /* -------------------------------------------------------------------------- */
# /*                              Build Properties                              */
# /* -------------------------------------------------------------------------- */

add_compile_options(-std=c++11)
add_definitions(-D_OS_UNIX)

# /* -------------------------------------------------------------------------- */
# /*                                Find Package                                */
# /* -------------------------------------------------------------------------- */

find_package(Eigen3 3.3.7 REQUIRED)
find_package(OpenCV REQUIRED )
find_package(realsense2 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  image_transport
  cv_bridge
)
catkin_package()
include_directories(
    include
    ${OpenCV_INCLUDE_DIRS}
    ${catkin_INCLUDE_DIRS})

# /* -------------------------------------------------------------------------- */
# /*                               Create Library                               */
# /* -------------------------------------------------------------------------- */

file(GLOB src_files src/*.cpp
                    src/Utils/*.cpp)

add_library(${PROJECT_NAME} SHARED ${src_files})

target_link_libraries(${PROJECT_NAME}
    Eigen3::Eigen
    ${OpenCV_LIBRARIES}
    ${catkin_LIBRARIES}
    jsoncpp
)

target_include_directories(${PROJECT_NAME} PUBLIC
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
    $<INSTALL_INTERFACE:include>
    PRIVATE src)

set_target_properties(${PROJECT_NAME} PROPERTIES
    VERSION ${PROJECT_VERSION}
    SOVERSION 1)



# /* -------------------------------------------------------------------------- */
# /*                            Executables                                     */
# /* -------------------------------------------------------------------------- */

add_executable(${PROJECT_NAME}_controller
  src/ros_nodes/controller_node.cpp
)
target_link_libraries(${PROJECT_NAME}_controller
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)


add_executable(${PROJECT_NAME}_task_planning_cartesian
  src/ros_nodes/lego_task_planning_cartesian_node.cpp
)
target_link_libraries(${PROJECT_NAME}_task_planning_cartesian
    ${PROJECT_NAME}
    ${catkin_LIBRARIES}
)

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})