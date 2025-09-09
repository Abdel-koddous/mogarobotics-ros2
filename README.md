### ROS2 mogarobotics_ws

To build mogabot2 moveit2 package run the following 2 commands:
```
cd mogarobotics-ros2
clear && colcon build && source install/setup.bash
ros2 launch mogabot2_moveit moveit.launch.py
```

Additionnaly, to make moveit2 topics traffic available to non-ROS applications (i.e robotic-arm-control-app...)

```
ros2 launch rosbridge_server rosbridge_websocket_launch.xml &
```