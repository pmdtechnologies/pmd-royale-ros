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

project (pmd_royale_ros_examples)

set (CMAKE_CXX_STANDARD 14)

find_package (rviz_common REQUIRED)
find_package (rclcpp REQUIRED)
find_package (std_msgs REQUIRED)
find_package (pluginlib REQUIRED)
find_package (Qt5 COMPONENTS Widgets REQUIRED)

# This setting causes Qt's "MOC" generation to happen automatically.
set (CMAKE_AUTOMOC ON)

# Define QT_NO_KEYWORDS
add_definitions (-DQT_NO_KEYWORDS)

include_directories ("${CMAKE_CURRENT_LIST_DIR}/include" "${Qt5Widgets_INCLUDE_DIRS}")

add_library (
    pmd_royale_ros_rviz_panel SHARED
    "${CMAKE_CURRENT_SOURCE_DIR}/include/PMDRoyaleRVIZ.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/PMDRoyaleRVIZ.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/CameraControlWidget.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CameraControlWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/CameraInfoWidget.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CameraInfoWidget.cpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/include/CameraParametersClient.hpp"
    "${CMAKE_CURRENT_SOURCE_DIR}/src/CameraParametersClient.cpp")
target_link_libraries (pmd_royale_ros_rviz_panel Qt5::Widgets)
ament_target_dependencies (pmd_royale_ros_rviz_panel "rclcpp" "std_msgs" "rviz_common" "pluginlib")

pluginlib_export_plugin_description_file (rviz_common plugins.xml)

install (TARGETS pmd_royale_ros_rviz_panel DESTINATION lib/${PROJECT_NAME})
install (DIRECTORY launch DESTINATION share/${PROJECT_NAME})
install (DIRECTORY rviz DESTINATION share/${PROJECT_NAME})
install (DIRECTORY config DESTINATION share/${PROJECT_NAME})
install (FILES plugins.xml DESTINATION lib/${PROJECT_NAME})

ament_package ()
