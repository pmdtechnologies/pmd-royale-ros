# pmd_royale_ros_driver Package

This package provides a ROS node for a time-of-flight (TOF) camera managed by pmd's Royale libraries.

ROS2 topics:
- `camera_info` : provide camera information
- `point_cloud` : PointCloud2 of ROS with 3 channels (x, y and z of Royale DepthData)
- `depth_image` : TYPE_32FC1 image. Looks like gray image if viewed in RViz. Points get brighter with distance.
- `gray_image`  : MONO8 image.

Node Parameters:
- `serial` : Serial number for a specific camera. If not set, the node connects to the first camera detected by Royale.
- `auto_exposure`: Option to enable auto exposure. Upon switching usecase, this value can change automatically.
- `exposure`: The camera's exposure time in microseconds. Must be within the minimum and maximum exposure time defined 
for the usecase. See this parameter's ParameterDescriptor for the exposure time range.

Read Only Node Parameters:
- `model` : The camera's name
- `available_usecases` : Usecases available for this camera. Ignored if provided as a parameter at startup and set
by the node at startup

# How to start node
Please see the pmd_royale_ros_examples package for example launch files to demonstrate ways to start the camera node.
