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
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    pmd_royale_ros_examples_path = get_package_share_directory('pmd_royale_ros_examples')
    print('[info]: pmd_royale_ros_examples_path={:s}'.format(pmd_royale_ros_examples_path))
    rviz_config = join(pmd_royale_ros_examples_path,'rviz', 'PMDRoyaleRVIZ.rviz')

    flexx_config = join(pmd_royale_ros_examples_path, 'config', 'flexx2.yaml')

    container = ComposableNodeContainer(
        name='pmd_royale_ros_camera_node_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        composable_node_descriptions=[
            ComposableNode(
                package="tf2_ros",
                name='hellotf2',
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
                    'rotation.w' : -0.5
                }]
            ),
            ComposableNode(
                package='pmd_royale_ros_driver',
                plugin='pmd_royale_ros_driver::CameraNode',
                name='pmd_royale_ros_camera_node',
                parameters=[flexx_config]
            )
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        arguments=['-d', rviz_config])
    return LaunchDescription([container, rviz_node])

