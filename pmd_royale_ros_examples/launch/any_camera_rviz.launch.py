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
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch_ros.actions import Node
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
from ament_index_python.packages import get_package_share_directory
from os.path import join

def generate_launch_description():
    pmd_royale_ros_examples_path = get_package_share_directory('pmd_royale_ros_examples')
    print('[info]: pmd_royale_ros_examples_path={:s}'.format(pmd_royale_ros_examples_path))
    rviz_config = join(pmd_royale_ros_examples_path,'rviz', 'PMDRoyaleRVIZ.rviz')
    nodeName_arg = DeclareLaunchArgument("nodeName", default_value="node")
    fullNodeName = ['pmd_camera_', LaunchConfiguration("nodeName")]
    pointcloudTopic = ["/", fullNodeName, "/point_cloud_0"]
    grayimageTopic = ["/", fullNodeName, "/gray_image_0"]
    depthimageTopic = ["/", fullNodeName, "/depth_image_0"]

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
                name = fullNodeName,
                parameters=[{
                    # # Uncomment below to set specific parameters
                    # 'serial' : '8230-93AE-1FA8-283C',
                    # 'usecase' : 'Mode_9_30fps',
                    # 'auto_exposure' : False,
                    # 'exposure_time': 230,
                    # 'access_code' : '',
                    # 'recording_file' : 'file_to_record_to.rrf',
                }]
            )
        ],
        output='screen',
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        output='screen',
        parameters=[{"pointcloudTopic": pointcloudTopic,
                     "grayimageTopic": grayimageTopic,
                     "depthimageTopic": depthimageTopic}],
        arguments=['-d', rviz_config],
    )
    return LaunchDescription([nodeName_arg, container, rviz_node])

