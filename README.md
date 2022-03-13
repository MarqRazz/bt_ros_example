# ROS2 Behavior Trees V3 Example

To build:
```
rosdep install -r --from-paths . --ignore-src --rosdistro $ROS_DISTRO -y
colcon build
```

To run node:
`ros2 run bt_ros_example ros_node_main`

To trigger subscriber:
`ros2 topic pub /some_msg std_msgs/msg/Empty {}`
