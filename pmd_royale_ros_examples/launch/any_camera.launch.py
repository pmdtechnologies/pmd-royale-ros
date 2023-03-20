# ****************************************************************************\
# * Copyright (C) 2023 pmdtechnologies ag
# *
# * THIS CODE AND INFORMATION ARE PROVIDED "AS IS" WITHOUT WARRANTY OF ANY
# * KIND, EITHER EXPRESSED OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE
# * IMPLIED WARRANTIES OF MERCHANTABILITY AND/OR FITNESS FOR A
# * PARTICULAR PURPOSE.
# *
# ****************************************************************************/

from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode

def generate_launch_description():
    container = ComposableNodeContainer(
        name='pmd_royale_ros_camera_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                plugin='tf2_ros::StaticTransformBroadcasterNode',
                parameters=[{
                    'frame_id' : 'pmd_royale_ros_camera_node_link',
                    'child_frame_id' : 'pmd_royale_ros_camera_node_optical_frame',
                    'translation.x' : 0.0,
                    'translation.y' : 0.0,
                    'translation.z' : 0.0,
                    'rotation.x' : 0.5,
                    'rotation.y': 0.5,
                    'rotation.z' : -0.5,
                    'rotation.w' : -0.5}]),
            ComposableNode(
                package='pmd_royale_ros_driver',
                plugin='pmd_royale_ros_driver::CameraNode',
                name='pmd_royale_ros_camera_node',
                parameters=[{
                    # # Uncomment below to set specific parameters
                    # 'serial' : '8230-93AE-1FA8-283C',
                    # 'usecase' : 'Mode_9_30fps',
                    # 'auto_exposure' : False,
                    # 'exposure_time': 230,
                }])
        ],
        output='screen',
    )

    return LaunchDescription([container])
