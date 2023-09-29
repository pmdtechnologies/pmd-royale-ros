#****************************************************************************
# Copyright (C) 2023 pmdtechnologies ag
#
# THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# PARTICULAR PURPOSE.
#
#****************************************************************************

cmake_minimum_required (VERSION 3.13)

project (pmd_royale_ros_driver VERSION 0.0.1 LANGUAGES CXX)

find_package (royale REQUIRED)

set (CMAKE_CXX_STANDARD 14)

find_package (ament_cmake REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (sensor_msgs REQUIRED)
find_package (rclcpp_components REQUIRED)

add_library (pmd_royale_ros_node SHARED "${CMAKE_CURRENT_SOURCE_DIR}/include/CameraNode.hpp"
                                        "${CMAKE_CURRENT_SOURCE_DIR}/src/CameraNode.cpp")
target_link_libraries (pmd_royale_ros_node royale::royale)
target_include_directories (pmd_royale_ros_node PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/include)
target_compile_definitions (pmd_royale_ros_node PRIVATE "COMPOSITION_BUILDING_DLL")
ament_target_dependencies (pmd_royale_ros_node "rclcpp" "std_msgs" "sensor_msgs"
                           "rclcpp_components")
rclcpp_components_register_nodes (pmd_royale_ros_node "pmd_royale_ros_driver::CameraNode")

install (TARGETS pmd_royale_ros_node
         ARCHIVE DESTINATION lib
         LIBRARY DESTINATION lib
         RUNTIME DESTINATION bin)

ament_package ()
