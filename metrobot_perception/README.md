# metrobot_perception

ROS package for sensors (e.g. depth and tracking cameras) and vision/tracking tasks.

## Cameras

We use the Intel RealSense 435I and T265 cameras for depth and tracking purposes. After installing the RealSense SDK 2.0 and `realsense-ros`, the 435I camera can be started with

```bash
roslaunch realsense2_camera rs_camera.launch camera:=camera1 filters:=pointcloud depth_width:=424 color_width:=424 depth_height:=240 color_height:=240 depth_fps:=15 color_fps:=15 initial_reset:=true
```
