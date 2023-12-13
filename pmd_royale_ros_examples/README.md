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

### Launch multiple camera nodes and rviz
It is possible to launch multiple camera nodes and work with them in the same rviz window. For this we assign a node name to each launched camera node.

Assuming you have n cameras, you will launch the first n-1 cameras like this:

```
ros2 launch pmd_royale_ros_examples any_camera.launch.py nodeName:=<NAME_OF_NODE>
```

The last camera will also launch rviz:
```
ros2 launch pmd_royale_ros_examples any_camera_rviz.launch.py nodeName:=<NAME_OF_LAST_NODE>
```

It is important to launch rviz with the last camera, because else it won't update 
for any cameras that are launched later. When you launch rviz the panel loads all 
camera nodes that are currently running. 

The rviz panel shows the images and pointcloud of the node named `node`, so if you change the node name for all of your nodes, you will have to set the correct image and pointcloud topics yourself. The same holds for switching between cameras. The whole upper part of the rviz window won't update automatically. Only the lower part with the `Control` and `Info` tabs does this. 