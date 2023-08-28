# Ros2 Camera Screen Navigation

## Installation
- Source Ros2 Humble `source /opt/ros/humble/setup.bash`
- Source Gazebo `source /usr/share/gazebo/setup.sh`
- Source your workspace `source <example_workspace>/install/setup.sh`
- Export models `export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:<example_workspace>/src/touch_navigation/models`
- Select Turtlebot model as waffle `export TURTLEBOT3_MODEL=waffle`
- Change `turtlebot3_waffle` folder in  `turtlebot3_gazebo/models` with new file in **RobotModelnURDF**
- Change `turtlebot3_waffle.urdf` file in `turtlebot3_description` with new file in **RobotModelnURDF**
## Usage
- For starting monitor independently from map `ros2 run touch_navigation monitor`
- For starting both gazebo and monitor `ros2 launch touch_navigation monitor_with_gazebo.launch.py`
- For starting just monitor `ros2 launch touch_navigation monitor_only.launch.py`

## Changing Sensor Resolution

For changing the sensor (depth or rgb camera) you can change the values inside width and height on the ***model.sdf*** file
```xml
<sensor name="intel_realsense_r200_depth" type="depth">
  <camera name="realsense_depth_camera">
    <image>
      <width>192</width>
      <height>1080</height>
    </image>
  </camera>
</sensor>
```
