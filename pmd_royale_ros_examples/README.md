# pmd_royale_ros_examples

This package contains launch files and examples for how to run the pmd TOF camera node. It also contains an RViz panel
to control the parameters of a running TOF camera node.

Please see the pmd_royale_ros_driver package for a list of node options and their descriptions.

## Examples:
### Launch a camera node only for any supported pmd TOF camera
```
ros2 launch pmd_royale_ros_examples any_camera.launch.py
```
Attempts to configure for the first unused camera that the pmd Royale library detects.

### Launch a camera node and rviz for any supported pmd TOF camera
```
ros2 launch pmd_royale_ros_examples any_camera_rviz.launch.py
```
Attempts to configure for the first unused camera that the pmd Royale library detects.

### Launch a specific camera node and rviz
This is an example for how to launch a specific Flexx2 camera by its serial number. It accepts a config file,
```config/flexx2.yaml``` with details about a specific camera. This is mainly for an example but to run this with your
specific Flexx2 camera, you must modify the ```serial``` parameter in ```config/flexx2.yaml```.
```
ros2 launch pmd_royale_ros_examples flexx2_rviz.launch.py
```
